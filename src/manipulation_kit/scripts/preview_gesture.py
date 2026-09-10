#!/usr/bin/env python3
"""Render a gesture CSV as a VIDEO of the full-body D1 (no robot needed).

Before running any gesture on the real arms — which are dangerous — watch the
exact commanded motion in sim and confirm it is what you expect:

    python3 scripts/preview_gesture.py joint_test_motion.csv joint_test.mp4
    # deps (host): pip install mujoco imageio[ffmpeg]

Loads ``manipulation_kit/description/d1/d1.urdf`` (the full-body primitives model — loads
anywhere, zero mesh assets), maps the CSV's shared joint contract (columns
R1..R7 = ArmSide::A = URDF ``Joint1_R..7_R``; L1..L7 = ArmSide::B = ``_L``;
degrees), interpolates the keyframes linearly over their durations, and writes
an MP4. What you see is EXACTLY the keyframe trajectory ``gesture_play`` will
interpolate on the robot (its runtime smoothing is gentler, never larger).

GUARD IN THE PREVIEW: every interpolated frame is ALSO run through pyguard
(the same limits/keep-out/self-collision model the robot-side validator uses).
If a frame violates, the video flashes red, the violation prints, and playback
STOPS there — exactly the stop you'd get on the robot. So an unsafe gesture is
visibly caught in the preview before anything touches hardware
(``gesture_play`` refuses such a CSV at pre-flight as well). ``--no-guard``
disables the check (pure visualization).
"""
from __future__ import annotations

import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
#: Package-relative anchors — assets ship inside manipulation_kit, so these
#: work from a checkout and from an installed wheel alike.
KIT = os.path.dirname(HERE)
DEFAULT_URDF = os.path.join(KIT, "description", "d1", "d1.urdf")
DEFAULT_HOME = os.path.join(KIT, "config", "home_pose.json")

CSV_COLS = [f"R{i}" for i in range(1, 8)] + [f"L{i}" for i in range(1, 8)]


def load_gesture_csv(path: str):
    """[(duration_s, [14 angles deg in CSV column order]), ...]"""
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or line.startswith("duration"):
                continue
            vals = [float(x) for x in line.split(",")]
            if len(vals) != 15:
                raise ValueError(f"bad row (want 15 cols): {line[:60]}")
            rows.append((vals[0], vals[1:]))
    if not rows:
        raise ValueError(f"no keyframes in {path}")
    return rows


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("csv", help="gesture CSV (omakaseos keyframe format)")
    p.add_argument("out", nargs="?", default=None, help="output .mp4 (default: <csv>.mp4)")
    p.add_argument("--urdf", default=DEFAULT_URDF)
    p.add_argument("--fps", type=int, default=25)
    p.add_argument("--size", type=int, nargs=2, default=(960, 720), metavar=("W", "H"))
    p.add_argument("--speed", type=float, default=1.0,
                   help="playback speed multiplier (2 = twice as fast video)")
    p.add_argument("--no-guard", action="store_true",
                   help="skip the per-frame pyguard check (pure visualization)")
    a = p.parse_args()
    out = a.out or os.path.splitext(a.csv)[0] + ".mp4"

    guard = None
    if not a.no_guard:
        from manipulation_kit.guard import MotionGuard
        guard = MotionGuard()

    import imageio
    import mujoco
    import numpy as np

    model = mujoco.MjModel.from_xml_path(a.urdf)
    model.vis.global_.offwidth = max(1280, a.size[0])
    model.vis.global_.offheight = max(960, a.size[1])
    model.vis.headlight.ambient[:] = [0.45, 0.45, 0.45]
    model.vis.headlight.diffuse[:] = [0.8, 0.8, 0.8]
    data = mujoco.MjData(model)

    # CSV column -> URDF joint qpos address (R_i -> Joint{i}_R, L_i -> Joint{i}_L)
    qadr = []
    for col in CSV_COLS:
        jname = f"Joint{col[1]}_{col[0]}"
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
        if jid < 0:
            raise RuntimeError(f"joint {jname} not in {a.urdf}")
        qadr.append(model.jnt_qposadr[jid])

    keys = load_gesture_csv(a.csv)
    renderer = mujoco.Renderer(model, height=a.size[1], width=a.size[0])
    opt = mujoco.MjvOption()
    for g in range(6):
        opt.geomgroup[g] = 1          # collision-only URDF: show all geom groups
    cam = mujoco.MjvCamera()
    cam.azimuth, cam.elevation, cam.distance = 115, -18, 2.6
    cam.lookat[:] = [0, 0, 0.45]

    writer = imageio.get_writer(out, fps=a.fps, codec="libx264", quality=7)
    prev = np.array(keys[0][1], dtype=float)
    n_frames = 0
    stopped = None
    for dur, target in keys:
        target = np.array(target, dtype=float)
        steps = max(1, int(round(dur * a.fps / a.speed)))
        for s in range(1, steps + 1):
            q = prev + (target - prev) * (s / steps)
            # the guard sees exactly what the robot would be commanded
            if guard is not None:
                rep = guard.check(joints_a_deg=list(q[:7]), joints_b_deg=list(q[7:]))
                if not rep.ok:
                    stopped = str(rep)
            for adr, deg in zip(qadr, q):
                data.qpos[adr] = np.deg2rad(deg)
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera=cam, scene_option=opt)
            frame = renderer.render()
            if stopped:
                # GUARD STOP: flash the offending pose red and freeze — this is
                # where the robot-side guard would halt the motion.
                red = frame.copy()
                red[..., 0] = np.minimum(255, red[..., 0].astype(int) + 110).astype(frame.dtype)
                red[..., 1] //= 2
                red[..., 2] //= 2
                for k in range(a.fps * 2):          # 2s of alternating flash
                    writer.append_data(red if (k // 3) % 2 == 0 else frame)
                break
            writer.append_data(frame)
            n_frames += 1
        if stopped:
            break
        prev = target
    writer.close()
    if stopped:
        print(f"GUARD STOP at frame {n_frames}: {stopped}")
        print(f"wrote {out} (playback halted at the violation — "
              "gesture_play would refuse this CSV at pre-flight)")
        return 2
    print(f"wrote {out}: {n_frames} frames @ {a.fps} fps "
          f"({n_frames / a.fps:.0f}s video, speed x{a.speed}); guard: clean")
    return 0


if __name__ == "__main__":
    sys.exit(main())
