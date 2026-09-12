> **PROSE ONLY — the code is not in this repository.**
>
> This is d1-sdk's `devices/omakase_arm/pyarmstate/README.md` @ `47a6337`,
> carried verbatim because it is the written record of how the D1 arm
> controller's states, faults and recovery actually behave — which mode you can
> enter from where, which faults are recoverable, and why the sequence is the
> sequence.
>
> The `pyarmstate` module itself did **not** move here: modes, recovery and
> state sequencing are `d1-firmwared`'s job by definition (they are sequences of
> commands against a live connection, which is exactly what manipulation-kit
> does not do). The `vendor_arm_sdk` / `ArmRobot` API in the examples below is
> retired; read them for the STATE MACHINE, not the calls. The daemon's REST API
> on `:4750` is what you actually talk to.

# pyarmstate

Pure-python (stdlib-only) state machine for the D1 arm controller: the single
place the arms' control-state transition and recovery procedure lives.
Consumers construct it instead of re-implementing the sequence; nothing changes
for code that does not opt in.

```python
from vendor_arm_sdk.robot import ArmRobot, DCSS
from pyarmstate import ArmStateMachine, FxTransport, DriveFaultError

robot = ArmRobot()
robot.connect(robot_ip="192.168.9.100")
arms = ArmStateMachine(FxTransport(robot, DCSS()))

arms.recover("A")                  # usable POSITION hold from any state
try:
    arms.try_torque("A")           # compliant mode, if the drives allow it
except DriveFaultError as exc:
    print("compliance unavailable:", exc)
    arms.recover("A")
```

## Controller behaviours absorbed here, and the measurements behind them

All numbers below were captured on the physical robot (Jetson host talking to
the arm controller at 192.168.9.100).

**Post-clear flapping.** After `clear_error()` on an arm latched in ERROR,
`cur_state` oscillates between IDLE(0) and ERROR(100) while `err_code` is
already 0. An adversarial run over 8 induced err6 recoveries measured flap
lengths of 0.175, 0.179, 0.181, 0.192, 0.295, 0.295, 0.298 and 0.300 s, with
17-25 state changes per flap, visiting only IDLE and ERROR. A single read
during that window is a coin toss, so every read is confirmed over a window
before it is believed. The default `confirm_window_s=0.5` clears the measured
worst case (0.300 s) by 0.2 s; the earlier 0.2 s default was refuted by the
same measurement (it can expire inside a flap lull and return a flap sample as
settled — `test_the_default_window_survives_the_longest_measured_flap` pins
this).

**err4, a POSITION request refused, for three independent reasons.**

1. *The arm was still physically moving.* State values do not reveal this: a
   compliant arm sagging under gravity reports a perfectly steady `cur_state`
   the whole time it drifts, spiking to 5-11 deg/s. Stillness is therefore
   measured from the controller's `low_speed_flag` AND the maximum absolute
   joint velocity (threshold `MAX_STILL_JOINT_VEL_DEG_S = 0.3`), and awaited
   before any mode request.
2. *Compliance settings survived the mode change.* Going to IDLE resets
   neither drag space nor the impedance type; only leaving drag does. An arm
   whose drag space is still set stalls at TRANS_TO_POSITION(101) with
   `err_code` 0 — a failure that does not even look like an error. Both are
   dropped (best-effort) before the request; an arm that was never compliant
   is unaffected.
3. *The commanded target itself was out of range.* Anchoring to where the arm
   rests re-commands exactly the pose that was refused, so retrying repeats
   the refusal indefinitely (reproduced 10/10 on hardware). Recovery
   re-anchors once to a caller-supplied reachable pose; without one it raises,
   because moving the arm somewhere nobody asked for is not a safe default.

**err6 (torque-entry request refused)** is the controller declining the request
in the state it was asked in — not a broken drive. Repeating the identical
request in the identical state gets the identical answer, so the REQUEST is
repeated only after the error is cleared, and a run of refusals raises
`DriveFaultError` naming the conditions that actually cause it rather than
spinning.

**err2 (servo failure)** is raised on the servo-engage path and can mean servo
power is absent. Recovery attempts once, then raises `ServoUnavailableError`
naming the likely cause rather than looping against dead hardware.

## Torque entry is one sequence, in two languages

`ArmStateMachine.try_torque()` and `Arm::enterTorqueMode()` in
`include/omakase_arm/arm.h` perform the SAME sequence, because one robot must
not have two. Its steps came from a vendor example that works on this hardware
plus measurements on the robot d1-1, and the four that are easiest to leave out
are the four that were:

- **wait for the arm to stop falling** once IDLE has removed servo power.
  Entering while it falls drifted 11.6 deg against 1.8 deg settled — and a
  transport whose `request_torque` is one of the vendor's composite helpers is
  refused outright, since those poll the low-speed flag over about 5 ms.
- **impose the reduced speed/acceleration ratio** the transition is expected to
  be made at, and **give the caller's ratio back** afterwards, on the failure
  path too.
- **clear the tool registration to an empty model before the transition, and
  register the real one after it.** The registration lives in the CONTROLLER and
  outlives the process, so without the first half this entry happens on top of
  whatever program ran last; and registering the real tool before the transition
  is itself what the controller declines.
- **refuse an entry whose commanded pose is far from the arm.** Torque engages
  against that pose at full authority with no easing; a stale target once ran an
  arm into the robot's own housing.

`tests/test_states_crosscheck.py` re-parses `arm.h` for each of the sequence's
numbers, so the two copies cannot drift.

## Layout

| File | Role |
| --- | --- |
| `states.py` | State/error constants and policy sets. Copies of values owned by `include/omakase_arm/robot_control.h` (states) and `sdk/vendor_arm_sdk/robot.py` (error table); `tests/test_states_crosscheck.py` re-parses both owners on every run so a drifted copy fails a test. |
| `machine.py` | `ArmStateMachine`: confirmed reads, `ensure_position` / `ensure_idle` / `try_torque` / `recover`, and the named failure exceptions. Talks to hardware only through a transport object. |
| `transport.py` | `FxTransport`: the one module that knows the vendor wrapper's API, and the only place the vendor's stage-then-commit protocol is applied (`clear_set` → setters → `send_cmd` → wait). A setter issued outside such a batch is not a weaker command; it is no command, because the next `clear_set` discards it. |
| `tool.py` | `ToolConfig` and `load_tool_config()`: the end effector's physical model, resolved from the same files and in the same order as `omakase_arm::ToolConfig::load()` in `arm.h`, so both languages give one answer for what is bolted to the flange. |
| `tests/` | Hardware-free suite: scripted fakes replaying captured controller behaviour, transport tests over a vendor double, a fake/real transport conformance check, and the constant cross-checks. |
| `tests/live_check.py` | On-robot check (not part of the automated suite): recovery from a latched fault to a confirmed POSITION hold, a torque-entry attempt, and recovery after the refusal. |

## Running the tests

```sh
cd devices/omakase_arm
PYTHONPATH= AMENT_PREFIX_PATH= python3 -m pytest pyarmstate/tests -q
```

No robot, vendor library, or third-party dependency is needed (pytest only).
The transport is exercised against a scripted double; the state machine against
timelines replaying the measured controller behaviour. On the robot,
`python3 pyarmstate/tests/live_check.py` runs the same procedure against the
real controller — read its module docstring first: it powers servos and must
only run with the arms clear of people and obstacles.
