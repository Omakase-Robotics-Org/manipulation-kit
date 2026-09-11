#!/usr/bin/env python3
"""Generate the ASSEMBLY-TEST gesture: wiggle each arm joint J7 -> J1 from HOME.

After assembling a D1 arm you want to confirm, joint by joint, that every axis
moves, moves in the right direction, and by the right amount — starting from the
WRIST (J7, least inertia, safest) and walking down to the SHOULDER (J1, moves the
whole arm). This script emits that test as a standard gesture CSV (omakaseos
keyframe format, degrees, first/last snapped to HOME), so the EXISTING, proven
playback path runs it on hardware with its safety pre-flight + runtime collision
guard:

    # 1. generate (host or robot; no hardware needed)
    python3 scripts/make_joint_test_gesture.py joint_test_motion.csv

    # 2. PREVIEW the exact motion as a video and WATCH it (host; needs mujoco):
    python3 scripts/preview_gesture.py joint_test_motion.csv joint_test.mp4

    # 3. only then, on the robot:
    ./example/gesture_play joint_test_motion.csv

Safety, in layers:
  - each wiggle is a small per-joint delta around HOME (default 8 deg, wrist 10),
    chosen to keep >=10 deg of margin to the joint limits at HOME;
  - one joint moves at a time; every excursion returns to HOME before the next;
  - EVERY generated keyframe is validated host-side with pyguard's MotionGuard
    (joint limits + torso keep-out + arm-arm + self-collision, the same model as
    the C++ validator plus the YUBI hand). A violating pose REFUSES to write;
  - gesture_play re-validates with its own safety_zones pre-flight gate and runs
    the collision-reactive guard during playback (belt and braces).

Joint order / sides follow the shared contract (GESTURES.md): CSV columns are
R1..R7 (ArmSide::A = physical LEFT arm, "_R" URDF tree) then L1..L7 (ArmSide::B).
"""
from __future__ import annotations

import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
#: Package-relative anchors. This script lives OUTSIDE the wheel (examples/),
#: so the anchor comes from the import rather than from this file's location;
#: the assets themselves still ship inside manipulation_kit, so this works from
#: a checkout and from an installed wheel alike.
KIT = os.path.dirname(os.path.abspath(__import__("manipulation_kit").__file__))
DEFAULT_URDF = os.path.join(KIT, "description", "d1", "d1.urdf")
DEFAULT_HOME = os.path.join(KIT, "config", "home_pose.json")

#: per-joint wiggle amplitude (deg), J1..J7. Wrist J7 gets a bit more (it is the
#: safest); everything keeps >=10 deg margin to the Lite limits at HOME
#: (J1/J3/J5 +/-173, J2 +/-118, J4 -145..+45, J6 +/-60, J7 +/-90).
DEFAULT_DELTAS = {1: 8.0, 2: 8.0, 3: 8.0, 4: 8.0, 5: 8.0, 6: 8.0, 7: 10.0}

#: seconds per segment at the default amplitude — a deliberately slow ~4 deg/s.
SEG_S = 2.0
#: dwell at HOME between joints, so the operator can tell the joints apart.
DWELL_S = 1.0


def load_home(path: str):
    """HOME as {"A": [J1..J7], "B": [J1..J7]} in degrees (SDK sides)."""
    d = json.load(open(path))
    vals = dict(zip(d["joint_order"], d["home_pose"]))
    return {"A": [vals[f"A{i}"] for i in range(1, 8)],
            "B": [vals[f"B{i}"] for i in range(1, 8)]}


def _fit_delta(guard, home, side: str, i: int, want: float):
    """Largest |delta| <= |want| (same sign) whose pose passes the guard, else 0.

    Some wiggles are genuinely tight at HOME (e.g. J2 inward swings the upper arm
    toward the belly keep-out) — rather than refuse the whole test, shrink THAT
    excursion to the largest safe amplitude and note it."""
    d = want
    while abs(d) >= 2.0:
        a, b = list(home["A"]), list(home["B"])
        (a if side == "A" else b)[i] = home[side][i] + d
        if guard.check(joints_a_deg=a, joints_b_deg=b).ok:
            return d
        d *= 0.5
    return 0.0


def build_keyframes(home, sides, deltas, seg_s: float, dwell_s: float, guard):
    """[(duration_s, A[7], B[7]), ...] — J7->J1 per side: HOME -> +d -> -d -> HOME.

    Every excursion is guard-FITTED: the requested amplitude shrinks (halving,
    min 2 deg) until the pose clears limits/keep-out/self-collision; an
    unclearable direction is skipped with a warning."""
    frames = [(0.0, list(home["A"]), list(home["B"]))]  # start AT HOME

    def add(dur, a, b):
        frames.append((dur, list(a), list(b)))

    for side in sides:
        for j in range(7, 0, -1):          # J7 first (wrist), J1 last (shoulder)
            i = j - 1
            for want in (+deltas[j], -deltas[j]):
                d = _fit_delta(guard, home, side, i, want)
                if d == 0.0:
                    print(f"  ~ skip arm {side} J{j} {'+' if want > 0 else '-'} "
                          "(no safe amplitude >= 2 deg)", file=sys.stderr)
                    continue
                if abs(d) < abs(want):
                    print(f"  ~ arm {side} J{j}: {want:+.0f} deg shrunk to "
                          f"{d:+.1f} deg (guard clearance)", file=sys.stderr)
                a, b = list(home["A"]), list(home["B"])
                (a if side == "A" else b)[i] = home[side][i] + d
                add(seg_s, a, b)
                add(seg_s, home["A"], home["B"])   # back to HOME after each excursion
            # dwell at HOME so the next joint's motion reads unambiguously
            add(dwell_s, home["A"], home["B"])
    return frames


def make_guard(urdf: str | None):
    from manipulation_kit.guard import MotionGuard

    return MotionGuard(urdf) if urdf else MotionGuard()


def validate(frames, guard) -> int:
    """pyguard-check every keyframe; returns the number of violating frames."""
    bad = 0
    for k, (_dur, a, b) in enumerate(frames):
        rep = guard.check(joints_a_deg=a, joints_b_deg=b)
        if not rep.ok:
            bad += 1
            print(f"  ! keyframe {k}: {rep}", file=sys.stderr)
    return bad


def write_csv(path: str, frames) -> None:
    with open(path, "w") as f:
        f.write("# D1 dual-arm gesture (omakaseos keyframe format, angles in DEGREES)\n")
        f.write("# joint order: R1..R7 (ArmSide::A = physical LEFT), L1..L7 (ArmSide::B = physical RIGHT)\n")
        f.write("# ASSEMBLY TEST: per-joint wiggle J7->J1 around HOME; "
                "generated by scripts/make_joint_test_gesture.py\n")
        f.write("duration,R1,R2,R3,R4,R5,R6,R7,L1,L2,L3,L4,L5,L6,L7\n")
        for dur, a, b in frames:
            row = [f"{dur:.4f}"] + [f"{v:.4f}" for v in a] + [f"{v:.4f}" for v in b]
            f.write(",".join(row) + "\n")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("out_csv", nargs="?", default="joint_test_motion.csv")
    p.add_argument("--arm", choices=["A", "B", "both"], default="both",
                   help="which arm to test (A = physical LEFT, B = RIGHT; "
                        "default both, sequentially)")
    p.add_argument("--delta-deg", type=float, default=None,
                   help="override the wiggle amplitude for ALL joints")
    p.add_argument("--seg-s", type=float, default=SEG_S,
                   help=f"seconds per wiggle segment (default {SEG_S})")
    p.add_argument("--home", default=DEFAULT_HOME, help="home_pose.json path")
    p.add_argument("--urdf", default=None,
                   help="URDF for the pyguard validation (default: pyguard's)")
    p.add_argument("--skip-guard", action="store_true",
                   help="skip the pyguard validation (NOT recommended)")
    p.add_argument("--unsafe-demo", action="store_true",
                   help="emit a DELIBERATELY UNSAFE gesture (arm A J2 swings into "
                        "the torso keep-out) to demonstrate the guard stop in "
                        "preview_gesture.py. NEVER play on hardware — "
                        "gesture_play refuses it at pre-flight anyway.")
    a = p.parse_args()

    if a.unsafe_demo:
        home = load_home(a.home)
        frames = [(0.0, list(home["A"]), list(home["B"]))]
        bad_a = list(home["A"])
        # +J2 swings the upper arm INTO the torso-belly keep-out (the guard-fit
        # run shrank +8 deg to +2 for exactly this reason); +30 deg is well past
        # the margin while still inside the +/-118 joint limit — so the demo
        # trips the COLLISION guard, not merely a limit clamp.
        bad_a[1] = home["A"][1] + 30.0
        frames.append((3.0, bad_a, list(home["B"])))
        frames.append((3.0, list(home["A"]), list(home["B"])))
        write_csv(a.out_csv, frames)
        print(f"wrote {a.out_csv}: UNSAFE guard-stop DEMO "
              "(preview flashes red + stops; do NOT play on hardware)")
        return 0

    deltas = dict(DEFAULT_DELTAS)
    if a.delta_deg is not None:
        deltas = {j: a.delta_deg for j in deltas}
    sides = ["A", "B"] if a.arm == "both" else [a.arm]

    home = load_home(a.home)
    guard = None if a.skip_guard else make_guard(a.urdf)
    if guard is None:
        # No guard: build with the raw deltas (a permissive stub fits everything).
        class _Pass:  # noqa: D401 — trivially-ok stub
            def check(self, **kw):
                class R: ok = True
                return R()
        frames = build_keyframes(home, sides, deltas, a.seg_s, DWELL_S, _Pass())
    else:
        frames = build_keyframes(home, sides, deltas, a.seg_s, DWELL_S, guard)
        bad = validate(frames, guard)   # belt-and-braces: re-check the final set
        if bad:
            print(f"REFUSING to write: {bad} keyframe(s) violate the motion guard",
                  file=sys.stderr)
            return 1
        print(f"guard: all {len(frames)} keyframes OK "
              "(limits + keep-out + self-collision)")

    write_csv(a.out_csv, frames)
    total = sum(f[0] for f in frames)
    print(f"wrote {a.out_csv}: {len(frames)} keyframes, ~{total:.0f}s, "
          f"arms={'+'.join(sides)}, J7->J1")
    return 0


if __name__ == "__main__":
    sys.exit(main())
