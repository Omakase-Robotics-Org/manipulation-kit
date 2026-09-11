#!/usr/bin/env python3
"""P0 verifier: does adding the LIFT to the IK chain extend the workspace?

Grid of right-TCP targets across heights 0.35..1.55 m (ground frame), fixed
tool orientation (the bent-elbow home orientation = gripper forward), solved
from the same home with three chains:

    arms       — 7-DoF arm only (today's baseline)
    arms+lift  — P0: arm + the 0.30 m prismatic lift
    full       — P1 preview: + differential-drive base (v, w)

Success = pos err < 5 mm AND ori err < 2 deg within `--iters` velocity-IK
steps. Also reports how much the lift/base moved — the mid-height rows check
the cost design (lift should stay ~still when the arm alone suffices).

Outputs: table on stdout, JSON next to this file, and an MP4 sweep video
(one representative column of targets, arms+lift vs arms side by side).
"""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mujoco  # noqa: E402
from wb_ik import WholeBodyIK  # noqa: E402

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
HOME = {"R": np.array([0.0, -0.6, 0.0, -1.2, 0.0, 0.4, 0.0]),
        "L": np.array([0.0, -0.6, 0.0, -1.2, 0.0, 0.4, 0.0])}
HEIGHTS = [0.35, 0.55, 0.75, 0.95, 1.15, 1.35, 1.55]
XS = [0.45, 0.60]
Y = 0.30                       # URDF arm "R" side (+y)
ITERS = 150


def ref_orientation():
    ik = WholeBodyIK("arms")
    ik.set_home(HOME)
    return ik.tcp_pose("R")[1]


def target_orientation(z):
    """Height-realistic tool orientation: how a human (or a grasp planner)
    would approach at that height — down-pitched for low targets, horizontal
    at torso height, up-pitched overhead. One FIXED world orientation across
    a 1.2 m span binds against the wrist limits (J6 ±60°, J7 ±90°) and turns
    the reach test into an orientation-feasibility test.
    Pitch is about world y: R_t = Rot_y(pitch) @ R_home."""
    if z < 0.60:
        pitch = np.radians(55.0)      # approach tilted down
    elif z < 0.90:
        pitch = np.radians(25.0)
    elif z < 1.25:
        pitch = 0.0                   # horizontal forward
    else:
        pitch = np.radians(-30.0)     # tilted up
    c, s = np.cos(pitch), np.sin(pitch)
    Ry = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return Ry @ ref_orientation()


def run_grid():
    rows = []
    for mode in WholeBodyIK.MODES:
        for z in HEIGHTS:
            for x in XS:
                ik = WholeBodyIK(mode)
                ik.set_home(HOME)
                res = ik.solve_multistart({"R": (np.array([x, Y, z]), target_orientation(z))}, iters=ITERS)
                ep, eo = res["err"]["R"]
                rows.append(dict(mode=mode, x=x, z=z, success=bool(res["success"]),
                                 iters=res["iters"], pos_mm=ep * 1e3, ori_deg=eo,
                                 lift=ik.lift_q(),
                                 base_disp=float(np.linalg.norm(ik.base_pose()[:2]))))
    return rows


def summarize(rows):
    print(f"{'mode':10s} {'z':>5s} {'x':>5s} {'ok':>3s} {'pos_mm':>7s} "
          f"{'ori_deg':>7s} {'lift':>7s} {'base_m':>7s}")
    for r in rows:
        print(f"{r['mode']:10s} {r['z']:5.2f} {r['x']:5.2f} "
              f"{'YES' if r['success'] else ' - ':>3s} {r['pos_mm']:7.1f} "
              f"{r['ori_deg']:7.2f} {r['lift']:+7.3f} {r['base_disp']:7.3f}")
    print()
    for mode in WholeBodyIK.MODES:
        sel = [r for r in rows if r["mode"] == mode]
        ok = sum(r["success"] for r in sel)
        print(f"{mode:10s}: {ok}/{len(sel)} targets reached")
    # cost-design check: mid-height targets should not recruit the lift
    mid = [r for r in rows if r["mode"] == "arms+lift" and 0.7 <= r["z"] <= 1.0]
    if mid:
        print(f"mid-height lift usage (want ~0): "
              f"max |lift| = {max(abs(r['lift']) for r in mid)*1000:.0f} mm")


def render_sweep(path):
    """Side-by-side sweep video: arms (left) vs arms+lift (right)."""
    import imageio.v2 as imageio
    frames_all = []
    for z in HEIGHTS:
        target = np.array([0.60, Y, z])
        R_t = target_orientation(z)
        per_mode = []
        for mode in ("arms", "arms+lift"):
            ik = WholeBodyIK(mode)
            ik.set_home(HOME)
            ren = mujoco.Renderer(ik.model, 480, 480)
            cam = mujoco.MjvCamera()
            cam.lookat[:] = [0.35, 0.15, 0.9]
            cam.distance, cam.azimuth, cam.elevation = 3.0, 155, -12
            snaps = []
            for it in range(400):
                res = ik.step({"R": (target, R_t)}, dt=0.02)
                if it % 6 == 0 or it == ITERS - 1:
                    mujoco.mj_forward(ik.model, ik.data)
                    ren.update_scene(ik.data, camera=cam)
                    scn = ren.scene
                    if scn.ngeom < scn.maxgeom:
                        g = scn.geoms[scn.ngeom]
                        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE,
                                            np.array([0.025, 0, 0]), target,
                                            np.eye(3).ravel(),
                                            np.array([1.0, 0.15, 0.15, 0.9], dtype=np.float32))
                        scn.ngeom += 1
                    snaps.append(ren.render().copy())
                ep, eo = res["err"]["R"]
                if ep < 5e-3 and eo < 2.0:
                    mujoco.mj_forward(ik.model, ik.data)
                    ren.update_scene(ik.data, camera=cam)
                    scn = ren.scene
                    if scn.ngeom < scn.maxgeom:
                        g = scn.geoms[scn.ngeom]
                        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE,
                                            np.array([0.025, 0, 0]), target,
                                            np.eye(3).ravel(),
                                            np.array([0.15, 1.0, 0.15, 0.9], dtype=np.float32))
                        scn.ngeom += 1
                    snaps.append(ren.render().copy())
                    break
            per_mode.append(snaps)
            ren.close()
        n = max(len(s) for s in per_mode)
        for i in range(n):
            l = per_mode[0][min(i, len(per_mode[0]) - 1)]
            r = per_mode[1][min(i, len(per_mode[1]) - 1)]
            frames_all.append(np.concatenate([l, r], axis=1))
        frames_all += [frames_all[-1]] * 12   # hold at each height
    imageio.mimsave(path, frames_all, fps=25, quality=8)
    print(f"video: {path} ({len(frames_all)} frames)")


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    rows = run_grid()
    summarize(rows)
    with open(os.path.join(OUT_DIR, "bench_p0.json"), "w") as f:
        json.dump(rows, f, indent=1)
    render_sweep(os.path.join(OUT_DIR, "p0_sweep_arms_vs_lift.mp4"))


if __name__ == "__main__":
    main()
