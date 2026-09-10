#!/usr/bin/env python3
"""Generate the IK ASSEMBLY-TEST gesture: small cartesian EE excursions from HOME.

The joint test (``make_joint_test_gesture.py``) exercises each axis in JOINT
space (FK). This is the IK half: from the HOME end-effector pose, move each hand
a few centimetres along +/-X, +/-Y, +/-Z (one axis at a time, returning to HOME
between excursions), with every waypoint solved by the SDK's collision-aware
numeric IK — so the test confirms the whole FK->IK->joints chain on the assembled
arm, not just the joints.

Pipeline (uses the existing offline binaries; no robot needed to GENERATE):
    fk_batch  (HOME joints -> EE anchor pose, vendor mm/deg XYZABC)
    ik_batch  (EE waypoints -> collision-free joint rows, status-flagged)
    pyguard   (belt-and-braces validation of every keyframe)
    -> standard gesture CSV -> preview_gesture.py -> gesture_play

NOT RUNNABLE FROM THIS REPO YET — see the TODO-RESEAT-ON-FIRMWARE-CLIENT block
below. fk_batch/ik_batch are d1-sdk C++ binaries over the vendor .so; they did
not move here. Point $D1_SDK_ARM_DIR at a built d1-sdk devices/omakase_arm to
run it in the meantime.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
#: Package-relative anchors — assets ship inside manipulation_kit, so these
#: work from a checkout and from an installed wheel alike.
KIT = os.path.dirname(HERE)
DEFAULT_URDF = os.path.join(KIT, "description", "d1", "d1.urdf")
DEFAULT_HOME = os.path.join(KIT, "config", "home_pose.json")

# --------------------------------------------------------------------------
# TODO-RESEAT-ON-FIRMWARE-CLIENT — this script does not run in this repo yet.
#
# It is the only script here that is NOT pure computation: it shells out to the
# d1-sdk C++ example binaries ``fk_batch`` and ``ik_batch``, which link the
# vendor ``libKine.so`` / ``the arm vendor's SDK shared library``. Neither the binaries, the vendor
# libs, nor the ``.VendorKinCfg`` kinematic configs came to manipulation-kit — they
# are hardware/vendor artefacts and belong to ``d1-firmwared``.
#
# It is carried anyway because the GESTURE it produces (the IK half of the
# assembly test — see docs/GESTURES.md) is real institutional work and the
# waypoint layout, the chained seeding, the status decoding and the guard
# validation are all still correct. What has to change is one seam: FK and IK
# must come from d1-firmwared's REST API (or, for FK, from
# manipulation_kit.arms) instead of from a subprocess.
#
# To reseat it:
#   1. replace fk_home_pose() with a call to the daemon's FK endpoint (or
#      manipulation_kit.arms.d1.arm FK, if the vendor libKine agreement
#      is not needed for this direction);
#   2. replace ik_batch_solve() with the daemon's batch IK, keeping the chained
#      seeding and the per-row status semantics;
#   3. delete ARM_DIR_FOR_VENDOR_BINARIES and the LD_LIBRARY_PATH plumbing.
# Until then the script raises a clear error rather than pretending.
#
# ``make_joint_test_gesture.py``, ``preview_gesture.py`` and ``ik_click_move.py``
# have NO such dependency and run today.
# --------------------------------------------------------------------------
#: A d1-sdk checkout with the example binaries built, if you still have one.
ARM_DIR_FOR_VENDOR_BINARIES = os.environ.get("D1_SDK_ARM_DIR", "")
FK_BIN = os.path.join(ARM_DIR_FOR_VENDOR_BINARIES, "example", "fk_batch")
IK_BIN = os.path.join(ARM_DIR_FOR_VENDOR_BINARIES, "example", "ik_batch")
IK_CONFIG = os.path.join(ARM_DIR_FOR_VENDOR_BINARIES, "config",
                         "vendor_kin.Cfg")

#: cartesian excursion from the HOME EE pose, millimetres, one axis at a time.
DEFAULT_OFFSET_MM = 30.0
SEG_S = 2.0      # seconds per segment (slow)
DWELL_S = 1.0    # dwell at HOME between axes

#: SDK ArmSide -> ik/fk_batch --arm index (ik_batch: 0 = left/A, 1 = right/B).
ARM_IDX = {"A": 0, "B": 1}


def load_home(path: str):
    d = json.load(open(path))
    vals = dict(zip(d["joint_order"], d["home_pose"]))
    return {"A": [vals[f"A{i}"] for i in range(1, 8)],
            "B": [vals[f"B{i}"] for i in range(1, 8)]}


def _lib_env():
    import platform
    arch = "aarch64" if platform.machine() == "aarch64" else "x86_64"
    lib = ":".join([os.path.join(ARM_DIR_FOR_VENDOR_BINARIES, "build", arch),
                    os.path.join(ARM_DIR_FOR_VENDOR_BINARIES, "lib"),
                    os.environ.get("LD_LIBRARY_PATH", "")])
    return {**os.environ, "LD_LIBRARY_PATH": lib}


def fk_home_pose(side: str, home_joints, config: str):
    """HOME EE pose (x,y,z mm, r,p,y deg) via fk_batch."""
    if not os.path.exists(FK_BIN):
        raise FileNotFoundError(f"{FK_BIN} missing — ./build.sh fk_batch")
    cmd = [FK_BIN, "--joints", *[f"{v:.6f}" for v in home_joints],
           "--arm", str(ARM_IDX[side]), "--config", config]
    out = subprocess.run(cmd, capture_output=True, text=True, check=True,
                         cwd=ARM_DIR_FOR_VENDOR_BINARIES, env=_lib_env()).stdout.strip().splitlines()
    rows = [l for l in out if "," in l and not l.lstrip().startswith("#")]
    return [float(x) for x in rows[-1].split(",")[:6]]


def ik_solve(side: str, poses, seed, config: str):
    """ik_batch: EE poses -> [(joints7, status), ...] (chained seeding)."""
    if not os.path.exists(IK_BIN):
        raise FileNotFoundError(f"{IK_BIN} missing — ./build.sh ik_batch")
    with tempfile.TemporaryDirectory() as td:
        inp, outp = os.path.join(td, "in.csv"), os.path.join(td, "out.csv")
        with open(inp, "w") as f:
            for p in poses:
                f.write(",".join(f"{v:.6f}" for v in p) + f",{ARM_IDX[side]}\n")
        cmd = [IK_BIN, "--in", inp, "--out", outp, "--arm", str(ARM_IDX[side]),
               "--config", config, "--seed", *[f"{v:.6f}" for v in seed]]
        subprocess.run(cmd, check=True, capture_output=True, text=True,
                       cwd=ARM_DIR_FOR_VENDOR_BINARIES, env=_lib_env())
        rows = []
        for line in open(outp):
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")
            rows.append(([float(x) for x in parts[:7]], parts[-1].strip()))
        return rows


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("out_csv", nargs="?", default="cartesian_test_motion.csv")
    p.add_argument("--arm", choices=["A", "B", "both"], default="both")
    p.add_argument("--offset-mm", type=float, default=DEFAULT_OFFSET_MM)
    p.add_argument("--home", default=DEFAULT_HOME)
    p.add_argument("--config", default=IK_CONFIG)
    a = p.parse_args()

    home = load_home(a.home)
    sides = ["A", "B"] if a.arm == "both" else [a.arm]

    # keyframes: (duration, A7, B7), starting AT HOME
    frames = [(0.0, list(home["A"]), list(home["B"]))]
    skipped = 0
    for side in sides:
        anchor = fk_home_pose(side, home[side], a.config)
        print(f"arm {side} HOME EE (mm/deg): "
              + ", ".join(f"{v:.1f}" for v in anchor))
        # +/- offset along each axis, one at a time, HOME between
        waypoints = []
        for axis in range(3):                      # x, y, z
            for sign in (+1, -1):
                w = list(anchor)
                w[axis] += sign * a.offset_mm
                waypoints.append(w)
        sols = ik_solve(side, waypoints, home[side], a.config)
        for w, (joints, status) in zip(waypoints, sols):
            if status != "ok":
                print(f"  ~ skip arm {side} EE {w[:3]}: IK status={status}",
                      file=sys.stderr)
                skipped += 1
                continue
            av, bv = list(home["A"]), list(home["B"])
            (av if side == "A" else bv)[:] = joints
            frames.append((SEG_S, av, bv))
            frames.append((SEG_S, list(home["A"]), list(home["B"])))
        frames.append((DWELL_S, list(home["A"]), list(home["B"])))

    # belt-and-braces: pyguard every keyframe (ik_batch already avoided collisions)
    from manipulation_kit.guard import MotionGuard
    guard = MotionGuard()
    bad = sum(1 for _d, av, bv in frames
              if not guard.check(joints_a_deg=av, joints_b_deg=bv).ok)
    if bad:
        print(f"REFUSING to write: {bad} keyframe(s) violate the motion guard",
              file=sys.stderr)
        return 1

    with open(a.out_csv, "w") as f:
        f.write("# D1 dual-arm gesture (omakaseos keyframe format, angles in DEGREES)\n")
        f.write("# joint order: R1..R7 (ArmSide::A = physical LEFT), L1..L7 (ArmSide::B = physical RIGHT)\n")
        f.write(f"# IK ASSEMBLY TEST: EE +/-{a.offset_mm:.0f}mm per axis from HOME; "
                "generated by scripts/make_cartesian_test_gesture.py\n")
        f.write("duration,R1,R2,R3,R4,R5,R6,R7,L1,L2,L3,L4,L5,L6,L7\n")
        for dur, av, bv in frames:
            f.write(",".join([f"{dur:.4f}"] + [f"{v:.4f}" for v in av]
                             + [f"{v:.4f}" for v in bv]) + "\n")
    total = sum(f[0] for f in frames)
    print(f"wrote {a.out_csv}: {len(frames)} keyframes, ~{total:.0f}s, "
          f"skipped {skipped} unreachable waypoint(s); guard OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
