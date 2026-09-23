# `probe` hardware trial on d1-2 — the gate for the contact verbs

The contact verbs (`Probe`, `Press`, `manipulation_kit.primitives.contact`) and
the executor capability under them (`move_until`) were built and tested
against kinematic mirrors and fake transports only. **Nothing in the kit has
touched a real surface yet.** This page is the procedure that decides whether
they may: Shu's hardware gate (decision 2, 2026-09-22).

| gate | acceptance |
|---|---|
| **G1 — controller health (the gate)** | **zero controller errors** over every trial on this page: no run ends `controller_fault`, `robot.state().faults()` is empty after every trial, both arms read `error_code == 0` and mode `position` at the end, and the daemon log shows no new arm error latch |
| G2 — contact is detected | 10 / 10 table probes: `report.completed`, the contact `made`, `stopped_by == "contact"` |
| G3 — accuracy (the target) | table z within **±3 mm** of the tape over the same 10 probes, every one (report mean, max \|error\| and standard deviation) |
| G4 — no false contact (proposed) | 3 / 3 probes in free air end `stopped_by == "max_travel"`, with the reported peak rise **below 2.0 Nm** (half the 4 Nm default threshold) |

G1 is the gate. G3 is the accuracy target the design carries; G2 and G4 are
what make G3 mean something (a probe that stops in the air measures nothing).
If any trial ends in a controller fault, **stop the trial**, clear the error on
the console (Clear error, then Home) and send the output as it is.

## What runs, and what does not

* **Position mode only.** `move_until` uploads the probe leg as an ordinary
  guarded trajectory (`POST /v1/arm/trajectory/start`), polls the moving arm's
  `GET /v1/arm/a/state` and the job's status at 50 Hz through the generated
  client, and cancels the job (`POST /v1/arm/trajectory/{id}/cancel`, "the
  arms hold the last accepted target") when a joint's `feedback_torque` has
  risen 4 Nm over what it read before the motion. No mode change, no torque
  command, no per-joint effort limit is sent.
* The leg travels at **10 mm/s** (`contact.PROBE_SPEED_M_S`). On the first
  rise past the threshold the command is frozen (the job is cancelled) while
  the rise is confirmed for 0.15 s with the arm stalled; a rise of 8 Nm
  (2x) is contact at once. The expected overshoot of the command past first
  touch is one poll: 0.2 mm of command, plus whatever the controller had
  queued.
* After contact the arm is re-commanded at the posture it MEASURED (a short
  trajectory from feedback to feedback), so it stops pushing and stays
  touching. The report is read from the measured joints, never the command.
* d1-firmwared **0.3.0** as served on d1-2 (document sha256 `388bcd08…`); the
  executor checks the document at connect time.

## 0. Before the robot

On the workstation, in a checkout of the branch under test:

```sh
python3.12 -m venv .venv && . .venv/bin/activate
pip install -e '.[firmware]' pytest
pytest -q tests/primitives/test_contact.py tests/executors/test_contact_executor.py
```

All green, or do not go further.

## 1. Set up

1. The JP wagon in front of d1-2 as for `examples/agent/scenes/d1-2_tape_cup.json`.
   Measure the wagon top **from the floor** with the tape at the spot the left
   hand will probe; write it down as `TABLE_FROM_FLOOR_M` (0.884 on
   2026-09-22).
2. Clear the wagon under the left hand; nothing loose within 10 cm of the
   probe spot.
3. Somebody at the e-stop for the whole trial.
4. With the console or teleop, put the **left** hand **50–60 mm** above the
   wagon at the probe spot, roughly fingertips-down. (The probe re-aims the hand at
   its current point; a hand far from pointing down means a large wrist turn
   before the leg, which the plan will show as many standoff steps. The
   script measures the start and stops with `"stopped": "start_posture"` when
   the hand is more than 20 deg from fingertips-down; it also stops at the
   first REFUSED probe instead of lifting after it.)
5. Release the console/teleop arm lease (the trial takes it at class
   `policy`, and an `operator` holder outranks it).

## 2. The trial script

Save as `probe_trial.py` next to the checkout and run it on a machine that
reaches the daemon. It prints one JSON line per trial and a summary.

```python
"""d1-2 probe trial — docs/probe-hardware-trial.md. Prints JSON lines."""
import json, math, sys
import numpy as np

from manipulation_kit.arms import get_arm_kinematics
from manipulation_kit.description.head_camera import floor_to_base_m
from manipulation_kit.executor import KinematicExecutor, run
from manipulation_kit.executors.firmware import FirmwareExecutor
from manipulation_kit.primitives import Nudge, Probe, record_contacts
from manipulation_kit.primitives.orientation import tool_from_link7
from manipulation_kit.world import ArmView, GripperView, WorldView

ROBOT = "http://d1-2:4750"
TABLE_FROM_FLOOR_M = 0.884          # <- the tape, at the probe spot
TABLE_TRIALS, AIR_TRIALS = 10, 3
MAX_START_TILT_DEG = 20.0          # the hand must start roughly fingertips-down
DRY = "--dry" in sys.argv           # kinematic mirror: plans only, no robot

kin = get_arm_kinematics("d1/arm", quiet=True)


def observe(robot, like=None):
    """The world as the executor measures it (+ what `like` already knew)."""
    state = robot.state()
    arms, grippers = [], []
    for side in ("left", "right"):
        arm = state.arms[side]
        kin.set_joints(side, arm.q)
        p, r = tool_from_link7(*kin.ee_pose(side))
        arms.append(ArmView(side, joints=arm.q, tool_p=p, tool_r=r,
                            mode=arm.mode, error_code=arm.error_code))
        hand = state.hands.get(side)
        grippers.append(GripperView(
            side, min(1.0, max(0.0, (hand.closedness if hand and
                                     hand.closedness is not None else 0.0))),
            holding=bool(hand and hand.holding),
            open_gap_m=None if hand is None else hand.open_gap_m))
    world = WorldView.of([] if like is None else list(like.objects),
                         arms=arms, grippers=grippers,
                         firmware_spec=getattr(robot, "firmware_spec", "") or "")
    return world if like is None else world.with_(contacts=like.contacts)


def health(robot):
    state = robot.state()
    return {"faults": state.faults(),
            "modes": {s: a.mode for s, a in state.arms.items()},
            "error_codes": {s: a.error_code for s, a in state.arms.items()}}


def lift_off(robot, world, dz):
    """Straight up by ``dz`` (on the nudge grid), so the next probe starts
    where this one did. A mirror touches nothing and would keep sinking, so
    ``--dry`` puts it back where it started instead."""
    if DRY:
        for side, q in START.items():
            kin.set_joints(side, q)
        return
    plan = Nudge(side="left", dz=dz, frame="base").plan(world, kin)
    assert plan.ok, plan
    report = run(plan, robot, kin=kin)
    assert report.completed, report.to_json()


START = {}


def trial(robot, world, kind, n, z_tape):
    probe = (Probe(side="left", direction="down", max_travel_m=0.12,
                   declare_as="table") if kind == "table" else
             Probe(side="left", direction="down", max_travel_m=0.03))
    plan = probe.plan(world, kin)
    if not plan.ok:
        # REFUSED: nothing moved, so nothing is lifted back either. The
        # caller stops on this line (d1-2, 2026-09-22: every probe was
        # refused and the lift after each one walked the hand to head height).
        return world, {"kind": kind, "n": n, "refused": True,
                       "plan": plan.to_json()}
    report = run(plan, robot, kin=kin)
    after = record_contacts(observe(robot, world), report, probe)
    c = after.contacts[-1] if report.contacts else None
    line = {"kind": kind, "n": n, "completed": report.completed,
            "stop_reason": report.stop_reason, "error": report.error,
            "verdict": probe.verifier(world)(after).to_json()["verdict"],
            **health(robot)}
    if c is not None:
        line.update(made=c.made, stopped_by=c.stopped_by,
                    z=round(float(c.p[2]), 5),
                    travel_mm=round(c.travel_m * 1000, 1),
                    peak_rise_nm=round(c.torque_nm, 3),
                    contact=report.contacts[-1].to_json())
        if kind == "table" and z_tape is not None:
            line["dz_mm"] = round((float(c.p[2]) - z_tape) * 1000, 2)
    return after, line


def main():
    robot_cm = (KinematicExecutor(kin) if DRY else
                FirmwareExecutor(base_url=ROBOT, vel_ratio=0.15))
    with (robot_cm if not DRY else _null(robot_cm)) as robot:
        z_tape = None
        if not DRY:
            lift = robot.lift_state().height_m
            z_tape = TABLE_FROM_FLOOR_M - floor_to_base_m(lift)
            print(json.dumps({"lift_m": lift, "z_tape_base": round(z_tape, 5),
                              "firmware_spec": robot.firmware_spec,
                              **health(robot)}))
        world = observe(robot)
        lines = []
        # THE START POSTURE IS CHECKED, not assumed: a hand far from
        # fingertips-down is a large in-place wrist turn the plan may refuse
        # (d1-2, 2026-09-22: 53 deg off after an Approach that missed). The
        # dry run starts from HOME on purpose and is not checked.
        axis = world.arm("left").tool_r.apply([0.0, 0.0, 1.0])
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, -float(axis[2])))))
        if not DRY and tilt > MAX_START_TILT_DEG:
            print(json.dumps({"stopped": "start_posture",
                              "tilt_from_down_deg": round(tilt, 1),
                              "limit_deg": MAX_START_TILT_DEG}))
            return
        # G4 first: free air. Up 50 mm (the hand starts 50-60 mm above the
        # wagon, so 100-110 mm), then 30 mm probes, each lifted back 30 mm.
        lift_off(robot, world, 0.05)
        world = observe(robot, world)
        START.update({s: kin.joints(s) for s in ("left", "right")})
        for n in range(AIR_TRIALS):
            world, line = trial(robot, world, "air", n, z_tape)
            print(json.dumps(line)); lines.append(line)
            if stop_here(line):
                return summary(lines)
            lift_off(robot, observe(robot, world), 0.03)
            world = observe(robot, world)
        # G1-G3: the first table probe travels 100-110 mm, every one after it
        # 50 mm (it starts where the 50 mm lift left it).
        for n in range(TABLE_TRIALS):
            world, line = trial(robot, world, "table", n, z_tape)
            print(json.dumps(line)); lines.append(line)
            if stop_here(line):
                break                       # G1: stop at the first fault
            lift_off(robot, world, 0.05)
            world = observe(robot, world)
        summary(lines)


def stop_here(line):
    """A fault, a stopped run, or a REFUSED plan ends the trial. A refused
    probe moved nothing; lifting after it only walks the hand upward."""
    return bool(line.get("faults") or line.get("stop_reason")
                or line.get("refused"))


class _null:
    def __init__(self, x): self.x = x
    def __enter__(self): return self.x
    def __exit__(self, *a): return False


def summary(lines):
    table = [l for l in lines if l["kind"] == "table" and "dz_mm" in l]
    dz = np.array([l["dz_mm"] for l in table]) if table else np.zeros(0)
    print(json.dumps({
        "G1_zero_controller_errors": all(not l.get("faults") and
                                         l.get("stop_reason") != "controller_fault"
                                         for l in lines),
        "G2_contacts": f"{sum(1 for l in lines if l['kind'] == 'table' and l.get('stopped_by') == 'contact')}/{TABLE_TRIALS}",
        "G3_dz_mm": {"mean": round(float(dz.mean()), 2) if dz.size else None,
                     "max_abs": round(float(np.abs(dz).max()), 2) if dz.size else None,
                     "std": round(float(dz.std()), 2) if dz.size else None,
                     "all_within_3mm": bool(dz.size == TABLE_TRIALS and np.all(np.abs(dz) <= 3.0))},
        "G4_air": [(l.get("stopped_by"), l.get("peak_rise_nm")) for l in lines if l["kind"] == "air"],
    }))


if __name__ == "__main__":
    main()
```

## 3. Run it

1. **Dry run first**, on the workstation: `python probe_trial.py --dry`
   (the kinematic mirror, from HOME). Every probe plans; the air and table
   probes all report `stopped_by: "max_travel"` (a mirror touches nothing),
   the verdicts are `false`, and the summary reads G1 true, G2 0/10. This
   proves the script and the plans, not the robot.
2. On d1-2: `python probe_trial.py | tee probe-trial-$(date +%Y%m%d-%H%M).jsonl`.
   The first line is the lift, `z_tape_base` and the health. Then a 50 mm
   lift, three air probes (30 mm each, ~3 s, each lifted back 30 mm), then
   ten table probes — the first travels 100–110 mm, every later one ~50 mm
   (~5 s) — each followed by a 50 mm lift.
3. Watch the first table probe with a hand near the e-stop. What should
   happen: the wrist turns fingertips-down (standoff), the hand descends
   slowly, stops on the wagon without a knock, and the lift follows.
4. Send the `.jsonl` file back, and the daemon log for the trial window
   (`journalctl -u d1-firmwared --since …`).

## 4. Reading the result

* `faults` non-empty or `stop_reason: "controller_fault"` anywhere → **G1
  fails.** Note which joint (`contact.joint`, 0-based) and its `peak_rise_nm`.
* An air probe with `stopped_by: "contact"` → a false contact: the free-motion
  torque of this arm at 10 mm/s crosses 4 Nm. Report `peak_rise_nm`; the
  threshold (`contact_nm`, model-bounded 1.5–6 Nm) or the speed has to move.
* A table probe with `stopped_by: "max_travel"` → the hand did not reach the
  wagon within 120 mm, or the rise never got to 4 Nm (check `travel_mm`).
* `dz_mm` consistently of one sign → a systematic offset (tool point, fingertip
  lead `grasp_geometry.PAD.lead_m` = 29 mm, lift offset, or the tape). A spread → the
  detection; `std` is the number to look at.
* `stopped_by: "guard"` → the daemon's own guard or slew gate aborted the job,
  or something else cancelled it; its message is in `contact.detail`.

## Result 2026-09-23 — the gate passed

d1-2, 2026-09-23 02:31Z, kit `feat/perceive-head` @ `92c6b78`, d1-firmwared
0.3.0 @ `897b796` (spec `a9c8b0d2…`), lift 0.205 m, tape at base z 0.166 m.
Log: `probe-trial-20260923-1031.jsonl`.

| gate | result |
|---|---|
| G1 zero controller errors | **true**: no fault, both arms `position`, error code 0 throughout |
| G2 table contacts | **10/10** `stopped_by: "contact"` |
| G3 contact height vs tape | mean **−2.13 mm**, max \|dz\| **2.26 mm**, σ **0.09 mm** (all within 3 mm) |
| G4 free air | 3/3 `max_travel`, largest rises **0.63–0.71 Nm** (threshold 4 Nm) |

Every table contact was on J1, rising **8.1–9.9 Nm** over its pre-motion
torque. A probe took **~5 s** (5.5–5.7 s from the second on, 8.4 s for the
first, which travels 78 mm instead of 50).

The −2.1 mm offset is systematic (σ 0.09 mm) — a tool-point / pad-lead /
lift-offset question, not detection noise — and is inside G3.

### Observed: the contact point walked sideways

The legs are straight down (the probe) and straight up (the +50 mm base-frame
Nudge after it), so the ten contacts should sit on one spot. They did not:

| | first contact | tenth contact | drift |
|---|---|---|---|
| x | 0.3937 | 0.3754 | −18.3 mm (−2.0 mm/cycle) |
| y | 0.0624 | −0.0259 | −88.3 mm (−0.1 mm/cycle for three cycles, then −13 mm/cycle) |

The air probes, each lifted back 30 mm, walked the same way (y 0.0886 →
0.0794 → 0.0715). Height (G3) was unaffected.

**Cause** (reproduced on the fake-daemon replay,
`tests/executors/test_probe_session_replay.py`: 73 mm in y at `92c6b78`):
every lift ended 8.2–8.5 mm to the side of its own line. The IK converges on
**Link7** within 2 mm / 0.05 rad, and the tool point is 100 mm further out, so
a posture the solver calls converged leaves the TOOL 7–9 mm off; re-solving
from it is a no-op, the knot counts as "exhausted inside the 12 mm path
window" and the leg walks on — always to the same side, because the null-space
pull toward READY decides which edge of the tolerance ball the solver stops
on. The next leg starts where that one ended, so the offsets add up. A second,
smaller term: the probe leg had 25 mm knots and the daemon interpolates joints
between them, so a probe that stops between two knots is 0.2 mm off the line
(again always the same way).

**Fix** (`redesign/fix-xy-drift`): a Nudge and every contact leg are
`Waypoint.exact` — solved to 0.2 mm / 1 mrad at Link7 (≤ 0.3 mm at the tool)
with the READY pull off, seeded knot by knot from the previous solution, and
held to the 3 mm arrival tolerance instead of the 12 mm transit window (or
refused). A contact leg is knotted every 5 mm. On the replay the ten
contacts now sit within **0.4 mm** of the first (was 74.9 mm), dz band
unchanged. To confirm on the robot: rerun the trial and check the xy spread
of the ten table contacts is ≤ 2 mm.

## 5. Not part of the gate (do afterwards, if G1–G3 pass)

* Three probes 50 mm apart under one `declare_as="table"`: the published
  `table` surface's normal should be within 1° of vertical on the level wagon
  and its face within 3 mm of every contact (`SurfaceMeasured`).
* One `Press(target=…, direction="forward")` on a fixed panel, `force_nm`
  4–6: the hand returns to its standoff (`ToolAt`) and G1 still holds.

## 6. Tip grasp trial — same session

`Grasp(contact="tip")` takes a flat thing between the finger TIPS instead of
the pad centres (`grasp_geometry.TIP`: 129 mm from the flange, nothing
leading, 2 mm clearance a side). **Until this trial passes, the kit's
default stays `contact="pad"`**, and the generated ROBOT FACTS tell the model
that `tip` is experimental and unmeasured on hardware.

**The descent finishes by contact (0.16.0, after the 2026-09-23 trial).** The
first trial stopped the tips at a fixed 14.9 mm over the wagon (3 mm + the
12 mm droop margin); the near arm did not sag and the jaws closed 7 mm above
an 8 mm slab, 3 of 3. A fingertip grasp onto a known surface now stops the
tips 15 mm up (`TIP_SEARCH_START_M`), then runs a `ContactStep` straight
down for at most 20 mm (5 mm past the modelled top), stopping at a 3.0 Nm
joint-torque rise (`TIP_CONTACT_NM`), and closes where the tips touched. The
droop margin no longer applies to it (keep setting it: pad grasps use it).
`plan.notes` says "SEARCHES down by contact"; `run(...).contacts[0]` is the
search's `ContactReport`.

| gate | acceptance |
|---|---|
| T1 — controller health | zero controller errors, as G1 |
| T2 — lifted | **10 / 10**: the `Grasp` verdict TRUE and a following `Lift(height_m=0.05)` verdict TRUE (`ObjectRose`) |
| T3 — the search touched | **10 / 10** grasps with `contacts[0].stopped_by == "contact"` and the touched tip height (`record_contacts(...)` → `ContactView.p[2]`) within ±5 mm of the tape-measured wagon top; a `max_travel` means the wagon is lower than the scene says — re-measure, do not raise the margin |
| T4 — no pad/table jam | zero **jam flags**: the jaws STALLED more than 4 mm open of the slab's presented width (they closed on the table, not the slab). A touch of the TIPS on the wagon is now expected (T3) and is not a flag |

The slab's orientation is declared **in the robot's base frame**, not as the
operator sees it. `yaw 0` = the slab's long side (`size[0]`) along the
robot's FORWARD axis (away from the chest); `yaw 1.5708` = long side along
the line through the robot's two shoulders. The 2026-09-23 photos were taken
from the robot's right side, where "left-right" is the forward axis: the slab
lay at yaw 0 while the script declared 1.5708, so the jaws (planned across
the declared 55 mm, along base x) spanned the real 91 mm. Check from BEHIND
the robot before every session.

About 15 minutes, right after the probe trial, same set-up:

1. The slab (a stack of business cards, 91 x 55 x 8 mm) flat on the wagon
   under the left hand at (x 0.403, y 0.10) in base, **long side across the
   robot (parallel to the shoulder line), `CARD_YAW_RAD = 1.5708`** — the
   orientation the left wrist can take at that spot (J7 about -48 deg). At
   yaw 0 the same spot is refused (`joint_limit`, coupled J7): turn the
   slab, do not re-declare it. Scene as before: `kind: object`,
   `size [L, W, T]`, the wagon top as a `surface` — the same file the loop
   uses, with d1-2's own calibration file
   (`~/.config/omakase/camera_calibration.json` on the robot, read by default).
2. Dry-run first (`tip_trial.py --dry`): every plan's notes must contain
   "SEARCHES down by contact", and the mirror reports
   `stopped_by="max_travel"` (it measures no contact — expected, and said).
3. Per trial: `Approach(contact="tip")`, `Grasp(contact="tip",
   grip="soft")`, `Lift(height_m=0.05)`, each planned, run with
   `run(plan, robot, kin=kin)` and verified from a fresh observation; fold
   the grasp's contacts into the world with `record_contacts(world, report,
   grasp)` and log the touched tip z, `stopped_by`, `torque_nm` and
   `travel_m`. The same PAD grasp on the slab must be REFUSED at planning
   (`object_too_flat`) — check it once, it is the control.
4. Stop at the first controller fault. Record the jaw gap at the stall per
   trial (T4) and film the grasp from behind the robot.

If T1-T4 pass, the default can move and the ROBOT FACTS line drops the
"experimental" warning (`agent/loop.py`, `robot_facts`). If the search stops
early on free air (contact before the tips reach the slab's height), the
3.0 Nm threshold is the number to revisit; if the pads jam, the 2 mm tip
clearance (`TIP_CLEARANCE_PER_SIDE_M`).

## What the kit needs from d1-firmware (to file)

* **A daemon-side stop condition on a trajectory** (e.g. `stop_if` with a
  per-joint `feedback_torque` rise over the pre-start value): evaluated in the
  daemon's 1 ms loop instead of one HTTP poll (20 ms) away. This is the
  latency that decides how far a probe presses before it stops.
* **The playback position in `TrajectoryStatus`** (sample index or the current
  setpoint): the kit infers where the command was frozen from `elapsed_ms`.
* **A tool-force estimate** (the controller's own, or joint torques through its
  Jacobian): `ContactCriterion.tool_force_n` is refused as `unmeasured` until
  the document carries one.
