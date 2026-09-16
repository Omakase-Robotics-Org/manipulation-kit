#!/usr/bin/env python3
"""Calibrate the wrist camera's REAL mount pose after installing the plate.

WHY. The plate's nominal camera frame (``description.py``:
``CAM_MOUNT_XYZ_M``, +15 deg pitch) is CAD truth for the plate, but the
assembly on a robot adds what CAD cannot know: bolt-hole clearance, the
camera module's seating in its bracket, the un-modelled lens offset inside
the no-nameplate UVC housing. Sim sensitivity analysis (d1-manip-sim
``sweep_sensitivity.py``) says extrinsic error is exactly the kind of
miscalibration that kills camera-guided grasping, so mounting a plate
should end with running this.

HOW — the gripper is its own calibration target. The wrist camera always
sees the claws (that is its purpose), and the claws' geometry in the
gripper ``base_link`` frame is EXACTLY known from ``description.py``:

    inner jaw faces at x = +/-(JAW_STROKE_M - q)   (q = jaw joint value)
    claw tips at      z = JAW_TIP_Z_M
    claw width        y = +/-CLAW_HALF_WIDTH_M

Sweep the jaws through a few openings, mark the claw tip corners in each
frame, and solve the 6-DoF camera pose that best reprojects them. No
checkerboard, no external fixture — the same idea as the real stack's
"measure the target through the same pipeline".

USAGE

    # 1. On the robot: capture wrist frames at several jaw values
    #    (d1-inference's shm reader, or any dump of the UVC stream), then
    #    mark the visible claw-tip corners per frame. Feature ids:
    #    {r|l}_tip_{inner|outer}_{top|bot} — see feature_points().
    python3 calibrate_wrist_camera_mount.py correspondences.json

    # 2. Self-test (no robot): synthesise observations through a camera
    #    perturbed by a KNOWN error, solve, and assert recovery.
    python3 calibrate_wrist_camera_mount.py --selftest

INPUT FORMAT (correspondences.json)

    {
      "intrinsics": {"fx": ..., "fy": ..., "cx": ..., "cy": ...},
      "observations": [
        {"jaw_q": 0.0,
         "pixels": {"r_tip_inner_top": [u, v], "l_tip_inner_bot": [u, v]}},
        ...
      ]
    }

OUTPUT: the solved camera pose in ``base_link`` (xyz + rpy), its delta from
the nominal mount, and a ``wrist_camera_calibration.json`` consumers load
instead of the nominal frame. Pure numpy — runs anywhere.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np

#: Geometry facts, DUPLICATED from ``description.py`` so this tool runs on
#: a bare robot python with no package install (importing the package pulls
#: in the CAN driver chain). tests/test_calibrate_wrist_camera_mount.py
#: asserts these against the package, so they cannot drift.
#: All three MEASURED on d1-3 2026-09-16 (Shu, callipers); they were
#: 0.035 / 0.14350 / 0.090 from the vendor CAD before that.
JAW_STROKE_M = 0.032
JAW_TIP_Z_M = 0.129

#: Claw body half-width across the jaw gap's transverse axis (base_link y),
#: measured off the vendored tcp mesh AABB (+/-0.019 in the jaw link frame).
CLAW_HALF_WIDTH_M = 0.019
#: Claw pad root, along +Z from the flange — the near end of the 58 mm pad
#: face (tips are JAW_TIP_Z_M, and the registered TCP is the tip).
PAD_ROOT_Z_M = 0.071

#: Nominal camera mount in base_link — MUST match description.py /
#: vendor_camera_plate.py. The OPTICAL frame is this mount rotated so
#: +Z looks out of the lens and image-down points at the fingers.
NOMINAL_XYZ = np.array([0.0, 0.079236, 0.014543])
NOMINAL_TILT_RAD = math.radians(15.0)


def nominal_optical_pose():
    """4x4 base_link -> optical-frame pose of the NOMINAL mount.

    Mount frame: +Z = face normal, 15 deg from flange +Z toward -Y; +Y up
    the plate arm. Optical: +Z out of the lens (along the face normal),
    +Y image-down toward the fingers = mount rotated pi about Z, matching
    vendor_camera_plate.py's ``wrist_camera_optical``.
    """
    c, s = math.cos(NOMINAL_TILT_RAD), math.sin(NOMINAL_TILT_RAD)
    R_mount = np.array([[1.0, 0.0, 0.0],
                        [0.0, c, -s],
                        [0.0, s, c]])
    R_flip = np.diag([-1.0, -1.0, 1.0])          # rot pi about local Z
    T = np.eye(4)
    T[:3, :3] = R_mount @ R_flip
    T[:3, 3] = NOMINAL_XYZ
    return T


def feature_points(jaw_q: float) -> dict:
    """Known claw-corner positions in base_link at jaw value ``jaw_q``.

    Eight corners: {r|l} claw x {inner tip, inner pad-root} x {+y|-y}.
    'inner' = the gap-facing face, the visually sharpest edge in a wrist
    frame. Ids match what the annotator marks.
    """
    xr = JAW_STROKE_M - jaw_q          # right claw inner face (base +x)
    out = {}
    for side, sx in (("r", +1.0), ("l", -1.0)):
        for zt, zname in ((JAW_TIP_Z_M, "tip"), (PAD_ROOT_Z_M, "root")):
            for wy, yname in ((+CLAW_HALF_WIDTH_M, "top"),
                              (-CLAW_HALF_WIDTH_M, "bot")):
                out[f"{side}_{zname}_inner_{yname}"] = np.array(
                    [sx * xr, wy, zt])
    return out


def _rodrigues(w):
    th = np.linalg.norm(w)
    if th < 1e-12:
        return np.eye(3)
    k = w / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * (K @ K)


def project(T_base_cam, K, pts_base):
    """Pinhole projection of Nx3 base-frame points through camera pose T."""
    T_cam_base = np.linalg.inv(T_base_cam)
    p = (T_cam_base[:3, :3] @ pts_base.T).T + T_cam_base[:3, 3]
    u = K["fx"] * p[:, 0] / p[:, 2] + K["cx"]
    v = K["fy"] * p[:, 1] / p[:, 2] + K["cy"]
    return np.stack([u, v], axis=1), p[:, 2]


def solve_pose(K, pts_base, uvs, T_init, iters=40):
    """Gauss-Newton over 6 params (rot vec delta + translation delta)."""
    T = T_init.copy()

    def residual(Tc):
        uv_hat, z = project(Tc, K, pts_base)
        if np.any(z <= 0.01):
            return None
        return (uv_hat - uvs).ravel()

    for _ in range(iters):
        r0 = residual(T)
        if r0 is None:
            raise RuntimeError("points behind the camera during solve")
        J = np.zeros((r0.size, 6))
        eps = 1e-5
        for i in range(6):
            dT = T.copy()
            if i < 3:
                w = np.zeros(3)
                w[i] = eps
                dT[:3, :3] = _rodrigues(w) @ T[:3, :3]
            else:
                dT[i - 3, 3] += eps
            r1 = residual(dT)
            J[:, i] = (r1 - r0) / eps
        try:
            step = np.linalg.lstsq(J, -r0, rcond=None)[0]
        except np.linalg.LinAlgError:
            break
        w, t = step[:3], step[3:]
        T[:3, :3] = _rodrigues(w) @ T[:3, :3]
        T[:3, 3] += t
        if np.linalg.norm(step) < 1e-10:
            break
    rms = math.sqrt(float(np.mean(residual(T) ** 2)))
    return T, rms


def rpy_of(Rm):
    """Fixed-axis XYZ rpy from a rotation matrix (URDF convention)."""
    sy = -Rm[2, 0]
    cy = math.sqrt(max(0.0, 1.0 - sy * sy))
    return (math.atan2(Rm[2, 1], Rm[2, 2]),
            math.atan2(sy, cy),
            math.atan2(Rm[1, 0], Rm[0, 0]))


def run_calibration(data: dict, out_path: str | None) -> dict:
    K = data["intrinsics"]
    pts, uvs = [], []
    for ob in data["observations"]:
        feats = feature_points(float(ob["jaw_q"]))
        for fid, uv in ob["pixels"].items():
            if fid not in feats:
                raise SystemExit(f"unknown feature id {fid!r}; "
                                 f"valid: {sorted(feats)}")
            pts.append(feats[fid])
            uvs.append(uv)
    pts = np.asarray(pts, dtype=float)
    uvs = np.asarray(uvs, dtype=float)
    if len(pts) < 6:
        raise SystemExit(f"only {len(pts)} correspondences; need >= 6 "
                         "(3+ features over 2+ jaw openings)")

    T_nom = nominal_optical_pose()
    T, rms = solve_pose(K, pts, uvs, T_nom)

    d_xyz = (T[:3, 3] - T_nom[:3, 3]) * 1000.0
    dR = T[:3, :3] @ T_nom[:3, :3].T
    d_ang = math.degrees(math.acos(np.clip((np.trace(dR) - 1) / 2, -1, 1)))
    result = {
        "xyz": T[:3, 3].tolist(),
        "rpy": list(rpy_of(T[:3, :3])),
        "reprojection_rms_px": rms,
        "delta_from_nominal_mm": d_xyz.tolist(),
        "delta_from_nominal_deg": d_ang,
        "n_points": int(len(pts)),
    }
    print(f"solved optical pose (base_link): xyz {np.round(T[:3, 3], 5).tolist()}")
    print(f"delta from nominal: {np.round(d_xyz, 2).tolist()} mm, "
          f"{d_ang:.3f} deg  (reprojection rms {rms:.2f} px, "
          f"{len(pts)} points)")
    if out_path:
        with open(out_path, "w") as fh:
            json.dump(result, fh, indent=1)
        print(f"-> {out_path}")
    return result


def selftest(noise_px: float = 0.5) -> bool:
    """Synthesise observations through a camera with a KNOWN installation
    error, solve from the nominal initial guess, and check recovery.

    This is the verifier: it proves the solver finds a realistic
    miscalibration (2 deg / 4 mm — bolt-clearance scale) to well under the
    error budget the sim sensitivity sweep allows, from marked pixels with
    +/-0.5 px annotation noise.
    """
    rng = np.random.default_rng(7)
    K = {"fx": 320.0, "fy": 320.0, "cx": 320.0, "cy": 240.0}

    T_true = nominal_optical_pose()
    w = rng.normal(size=3)
    w = w / np.linalg.norm(w) * math.radians(2.0)      # 2 deg tilt error
    T_true[:3, :3] = _rodrigues(w) @ T_true[:3, :3]
    t_err = rng.normal(size=3)
    T_true[:3, 3] += t_err / np.linalg.norm(t_err) * 0.004   # 4 mm offset

    data = {"intrinsics": K, "observations": []}
    for q in (0.0, 0.012, 0.025):
        feats = feature_points(q)
        uv, _ = project(T_true, K, np.array(list(feats.values())))
        uv = uv + rng.normal(scale=noise_px, size=uv.shape)
        data["observations"].append({
            "jaw_q": q,
            "pixels": {fid: uv[i].tolist()
                       for i, fid in enumerate(feats)},
        })

    result = run_calibration(data, out_path=None)
    T_sol = np.eye(4)
    T_sol[:3, 3] = result["xyz"]
    err_mm = np.linalg.norm(np.array(result["xyz"]) - T_true[:3, 3]) * 1000
    # angle recovery: compare against the true rotation
    cr, sr = [f(a) for a in result["rpy"] for f in (math.cos, math.sin)][:0] or (None, None)
    Rx = _rodrigues(np.array([result["rpy"][0], 0, 0]))
    Ry = _rodrigues(np.array([0, result["rpy"][1], 0]))
    Rz = _rodrigues(np.array([0, 0, result["rpy"][2]]))
    R_sol = Rz @ Ry @ Rx
    dR = R_sol @ T_true[:3, :3].T
    err_deg = math.degrees(math.acos(np.clip((np.trace(dR) - 1) / 2, -1, 1)))
    ok = err_mm < 1.0 and err_deg < 0.2
    print(f"selftest: injected 2 deg / 4 mm, recovered to {err_mm:.2f} mm / "
          f"{err_deg:.3f} deg with {noise_px} px pixel noise -> "
          + ("OK" if ok else "FAILED"))
    return ok


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Solve the wrist camera's real mount pose from marked "
                    "claw corners (the gripper is its own target).")
    ap.add_argument("correspondences", nargs="?",
                    help="JSON of intrinsics + per-jaw-value pixel marks")
    ap.add_argument("--out", default="wrist_camera_calibration.json")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        return 0 if selftest() else 1
    if not args.correspondences:
        ap.error("give a correspondences JSON or --selftest")
    data = json.load(open(args.correspondences))
    run_calibration(data, args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
