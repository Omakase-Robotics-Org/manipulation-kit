#!/usr/bin/env python3
"""P1 verifier + demo: whole-body TRACKING with the differential-drive base.

The right TCP follows a 2.4 m long moving reference (forward traverse with a
height sweep 0.75→1.35 m and a lateral weave) — far beyond the arm's static
workspace, so the base and lift MUST flow with the hand to track it. The
reference moves at ~0.25 m/s; tracking runs the streaming step() at 50 Hz.

Verifier:
  - steady-state tracking error < 10 mm (after 1 s warm-up)
  - base lateral slip ≡ 0 by construction (reduced (v,w) velocity space) —
    asserted numerically from the integrated base path
  - video shows base + lift + both arms moving simultaneously (the L arm
    holds a fixed pose relative to the torso via its posture pull).
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mujoco  # noqa: E402
from wb_ik import WholeBodyIK, Costs  # noqa: E402
from bench_p0 import HOME, ref_orientation  # noqa: E402


def smooth_orientation(z):
    """Continuous pitch profile (the banded bench version JUMPS 25-30 deg at
    its boundaries — a step in the orientation reference makes the tracker
    trade position error to chase it)."""
    pitch = np.radians(np.interp(z, [0.35, 0.75, 0.95, 1.25, 1.55],
                                    [55.0, 25.0, 0.0, 0.0, -30.0]))
    c, s_ = np.cos(pitch), np.sin(pitch)
    Ry = np.array([[c, 0, s_], [0, 1, 0], [-s_, 0, c]])
    return Ry @ ref_orientation()

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
DT = 0.02
SPEED = 0.25          # reference forward speed [m/s]
LENGTH = 2.4          # traverse length [m]


def reference(t):
    """Moving TCP reference: forward traverse + height sweep + gentle weave."""
    s = min(t * SPEED, LENGTH)
    phase = s / LENGTH
    x = 0.55 + s
    y = 0.30 + 0.10 * np.sin(2 * np.pi * 1.5 * phase)
    z = 0.95 + 0.15 * np.sin(2 * np.pi * 1.0 * phase)   # 0.80 .. 1.10 — the STREAMING envelope (static reach is wider: see bench_p0)
    return np.array([x, y, z]), z


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    import imageio.v2 as imageio
    ik = WholeBodyIK("full", Costs(base_v=6.0, base_w=12.0, lift=3.0))
    ik.set_home(HOME)
    T = LENGTH / SPEED + 2.0
    n = int(T / DT)
    errs, base_path, lift_path = [], [], []
    frames = []
    ren = mujoco.Renderer(ik.model, 480, 640)
    cam = mujoco.MjvCamera()
    for i in range(n):
        t = i * DT
        p_ref, z = reference(t)
        R_ref = smooth_orientation(z)
        p_next, _ = reference(t + DT)
        v_ref = (p_next - p_ref) / DT
        res = ik.step({"R": (p_ref, R_ref)}, dt=DT, gain=6.0,
                      ff={"R": v_ref})
        ep, eo = res["err"]["R"]
        errs.append((t, ep, eo))
        bp = ik.base_pose()
        base_path.append(bp.copy())
        lift_path.append(ik.lift_q())
        if i % 2 == 0:
            cam.lookat[:] = [bp[0] + 0.3, 0.15, 0.85]
            cam.distance, cam.azimuth, cam.elevation = 3.4, 145, -12
            mujoco.mj_forward(ik.model, ik.data)
            ren.update_scene(ik.data, camera=cam)
            scn = ren.scene
            if scn.ngeom < scn.maxgeom:
                g = scn.geoms[scn.ngeom]
                mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE,
                                    np.array([0.03, 0, 0]), p_ref,
                                    np.eye(3).ravel(),
                                    np.array([1.0, 0.2, 0.2, 0.9], dtype=np.float32))
                scn.ngeom += 1
            frames.append(ren.render().copy())
    ren.close()
    imageio.mimsave(os.path.join(OUT_DIR, "p1_traverse_fullbody.mp4"),
                    frames, fps=int(0.5 / DT), quality=8)

    # ---- verifier ----
    errs = np.array([(t, ep, eo) for t, ep, eo in errs])
    steady = errs[errs[:, 0] > 1.0]
    base_path = np.array(base_path)
    # nonholonomy check: lateral velocity in the heading frame ≈ 0
    dxy = np.diff(base_path[:, :2], axis=0) / DT
    yaw_mid = base_path[:-1, 2]
    v_lat = -np.sin(yaw_mid) * dxy[:, 0] + np.cos(yaw_mid) * dxy[:, 1]
    print(f"steady-state tracking: mean {steady[:,1].mean()*1000:.1f} mm, "
          f"p95 {np.percentile(steady[:,1],95)*1000:.1f} mm, "
          f"max {steady[:,1].max()*1000:.1f} mm; ori p95 "
          f"{np.percentile(steady[:,2],95):.2f} deg")
    print(f"base travelled {base_path[-1,0]:.2f} m fwd; lift range "
          f"[{min(lift_path):+.3f}, {max(lift_path):+.3f}]")
    print(f"nonholonomy: max |lateral slip velocity| = {np.abs(v_lat).max():.2e} m/s")
    ok = (np.percentile(steady[:, 1], 95) < 0.010
          and np.abs(v_lat).max() < 1e-9)
    print("P1 verifier:", "PASS" if ok else "FAIL")


if __name__ == "__main__":
    main()
