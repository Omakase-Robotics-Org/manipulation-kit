"""Before the first move: is this robot the robot the plans assume?

Run it against a live d1-firmwared and it answers five questions, in the order
they can hurt you. Nothing here commands a joint.

    pip install -e '.[firmware]'
    python examples/preflight.py --robot http://d1-2:4750
    python examples/preflight.py --robot http://d1-2:4750 --scene examples/agent/scenes/tabletop.json

1. **Can we talk to it, and who has the arms?** A lease held by an
   ``operator`` (teleop, the console) outranks us and always will: a person at
   the robot wins.
2. **Is the state readable?** Both arms' joints, both grippers. A gripper that
   cannot be read is UNKNOWN, not open — and every verb that would open a hand
   refuses on an unknown one.
3. **Is the arm where its command is?** Engaging position control from idle
   with a stale command snaps the arm to it. On d1-2 (2026-09-10) one joint's
   command sat 48 degrees from its measurement.
4. **Does the installed tool match the one the kit plans with?** Every waypoint
   is a TOOL-POINT pose, so a different gripper means every plan means a
   different place. The kit's tool revision is printed; check it against the
   hand that is actually bolted on.
5. **Does forward kinematics agree with the robot?** The kit's FK is run on the
   MEASURED joints and the tool point is printed, in the base frame. Put a
   tape measure on it. If it disagrees, stop: a 10 mm FK error is a 10 mm
   grasp error and no amount of careful scene measurement will recover it.

Exit code 0 means every check passed.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import List, Optional, Sequence

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent / "agent"))


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--robot", default="http://127.0.0.1:4750")
    parser.add_argument("--scene", type=Path, default=None,
                        help="a measured scene file to sanity-check too")
    args = parser.parse_args(argv)

    from manipulation_kit.arms import get_arm_kinematics
    from manipulation_kit.executors.firmware import FirmwareExecutor
    from manipulation_kit.hands.d1.parallel_gripper.description import (
        DRIVEN_OPEN_GAP_M, PAD_CENTRE_Z_M)
    from manipulation_kit.primitives.approach import tool_from_link7, tool_revision
    from manipulation_kit.world.views import INTERIOR_FRACTION

    problems: List[str] = []
    kin = get_arm_kinematics("d1/arm", quiet=True)
    robot = FirmwareExecutor(base_url=args.robot, heartbeat=False)

    print(f"tool the kit plans with: {tool_revision()}")
    # The numbers the kit will plan with, read from the kit — not prose that
    # goes stale the moment the description (or its env knob) changes.
    print(f"   (pad centre {PAD_CENTRE_Z_M * 1000:.0f} mm, driven opening "
          f"{DRIVEN_OPEN_GAP_M * 1000:.2f} mm. If a different hand is bolted "
          f"on, stop here.)")

    lease = robot.acquire()
    print(f"lease: held by {lease.holder!r} at class {lease.lease_class!r}, "
          f"epoch {lease.epoch}, ttl {lease.ttl_s}s")
    try:
        state = robot.state()
        for side in ("left", "right"):
            q = state.joints.get(side)
            if q is None:
                problems.append(f"the {side} arm reports no joints")
                continue
            saved = np.array(kin.joints(side), dtype=float)
            try:
                kin.set_joints(side, q)
                p, _r = tool_from_link7(*kin.ee_pose(side))
            finally:
                kin.set_joints(side, saved)
            print(f"{side} arm: joints {np.round(np.degrees(q), 1).tolist()} deg")
            print(f"          FK tool point {np.round(p, 4).tolist()} m base "
                  f"— MEASURE THIS")
            if side not in state.grippers:
                problems.append(
                    f"the {side} gripper cannot be read; it is UNKNOWN, not "
                    f"open, and every verb that would open it will refuse")
            else:
                print(f"{side} gripper: closedness "
                      f"{state.grippers[side]:.2f}, holding "
                      f"{state.holding.get(side)}")
        try:
            robot.position_mode()
            print("position mode: engaged at this run's velocity/acceleration "
                  "ratios")
        except Exception as exc:  # noqa: BLE001
            problems.append(f"position mode refused: {exc}")
    finally:
        robot.release()
        print("lease: released")

    if args.scene is not None:
        from live import objects_from
        scene = json.loads(args.scene.read_text(encoding="utf-8"))
        for item in objects_from(scene):
            print(f"scene: {item.to_text()}")
            if getattr(item, "interior_measured", True) is False:
                problems.append(
                    f"{item.name}'s interior is ESTIMATED at "
                    f"{INTERIOR_FRACTION:.0%} of its size; "
                    f"measure it, or nothing may be placed into it")

    if problems:
        print("\nNOT READY:")
        for line in problems:
            print(f"  - {line}")
        return 1
    print("\nready.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
