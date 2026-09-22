# The primitive contract

Every verb in `manipulation_kit.primitives` keeps the same three-part contract.
That uniformity is the whole value of the package: a consumer — a script, a
teleop assist, a collection macro, an LLM loop — can treat any verb like any
other, and adding a tenth verb costs no consumer a change.

```python
preconditions(world) -> list[Unmet]
plan(world, kin)     -> Plan | PlanError      # pure
verifier(world0)     -> Verifier              # measured
```

## 1. `preconditions(world) -> list[Unmet]`

Cheap. No kinematics, no IK, no guard. It answers *is this verb applicable as
asked*, which is a different question from *can the arm get there*:

* `no_such_object` — and the `Unmet.remedy` lists what the world does hold
* `frame_stale` / `unknown_frame` — the pose exists but cannot be resolved
* `not_holding` / `already_holding` — the hand is in the wrong state
* `object_too_wide` — the driven jaws open 51.96 mm and this is wider
* `bad_side`, `bad_approach`, `bad_grip`, `bad_frame`, `no_motion` — the
  arguments themselves

Keeping these separate from the reach problem matters because the two want
different reactions. A misnamed object needs a different word; an unreachable
one needs a different approach, a different hand, or a nudge.

## 2. `plan(world, kin) -> Plan | PlanError`

**Pure.** It borrows the kinematic model, poses it to solve, and puts it back
exactly as it found it — on success and on every refusal. Planning twice from
the same world gives the same plan.

**Fully checked before anything moves.** Each pose waypoint is split into
per-tick knots bounded by `safety.MAX_STEP_M` / `MAX_STEP_RAD`, and each knot
goes through `GuardedArm.solve_ee`: damped-least-squares IK with the joint
limits enforced *inside* the iteration, a forward-kinematics re-check that the
solution reaches the pose it was asked for, the per-tick joint-step clamp, and
the `MotionGuard` collision check over the **two-arm** posture. The knots exist
because the solver is local: from HOME, a single call to a top-down grasp pose
lands in a local minimum and returns `None` even though random restarts solve
it. Walked in small steps, every knot converges — and the knots are also
exactly the 50 Hz targets the robot wants.

**A refused knot is routed around, not reported.** Interpolation needs
somewhere to go when a knot is refused: on the `blocks-eval` wagon, a top-down
grasp has a standoff and a grasp pose that are both guard-*clean* (36 mm of
body clearance) and a straight line from HOME that puts `Link4_R` inside
`torso_belly` at knot 1–3. Refusing there deleted the grasp from an agent's
menu — 61 of 61 turn-0 approaches — for a goal the arm can hold. So a rejected
waypoint is retried through `planning.VIA_OFFSETS_M`: a short, fixed,
cheapest-first list of clearance points (lift the hand up and out from the
torso, keep its orientation, then travel), each re-solving **both** legs, and
then through a re-seed from the arm's searched `ready()` posture. The candidate
list is measured, not guessed — 144 candidates swept against 37 grasps across
the wagon, two cover all 33 that can be planned at all. A plan that detoured
says so in `Plan.notes`. Nothing else changes: when no candidate works the
refusal is the straight line's own, with the same reason, waypoint index and
residual, and `solve_path(..., allow_via=False)` is the straight line alone.

**The jaws are squared to the object, not to the base frame.** `Approach`'s
roll comes from `ObjectView.footprint_axis` — the long horizontal axis where
there is one, the widest where the footprint is square. A square prism has no
*preferred* grasp and it does have a wrong one: a 40 mm cube yawed 11.7° is
47.3 mm across base-aligned jaws, past the 43.96 mm the driven gripper can
take, so the pads meet two corners and stall holding nothing.

**The tool point is the pad CENTRE, and the pads reach 29 mm past it.** A
top-down `Grasp` is therefore raised to keep the finger tips clear of whatever
the object is standing on — the MEASURED surface under it when one is known,
its own underside only when not (`grasp_geometry.grasp_pose`,
`SUPPORT_CLEARANCE_M` 3 mm)
— descending to a 40 mm cube's centre asks for the tips 9 mm *under the
table*, which jams the fingers and stops the arm 17 mm high and 19 mm to the
side. The pads are 58 mm deep, so the raised grasp still has 37 mm of pad
against the cube. An object too flat for the tips to reach beside is refused
(`object_too_flat`) rather than grasped over.

**A refusal is a typed value, never a silent no-op.** `PlanError` carries the
reason, the waypoint index and label, and the residual:

```
grasp refused: guard_reject at waypoint 1 (grasp), 19 mm short —
the motion guard refused the left arm's posture at 'grasp' — it would hit the
body, the other arm or itself
```

The reason vocabulary is closed (`PLAN_REASONS`): `ik_fail`, `infeasible`,
`guard_reject`, `unreachable_object`, `unreachable_destination`,
`no_such_object`, `frame_stale`, `unknown_frame`, `precondition_unmet`,
`learned_policy_required`. A consumer switches on it; it never parses a
message.

`unreachable_destination` is the one worth reading twice, because it is a
different KIND of answer. `ik_fail` names one waypoint the solver could not
reach, and a caller's sensible response is a different waypoint.
`unreachable_destination` means `Carry`/`Place` already tried the whole
transit ladder (`CARRY_CLEARANCE_LADDER_M`, 100 → 40 mm above the destination's
rim, floored at what the carried object needs to clear it) and this **arm**
cannot get over this **destination** at any of them. It carries the smallest
residual seen and the rung that came closest. The answer is the other arm, or
a nearer destination — never a smaller clearance, which is why the floor is
part of the primitive rather than part of the caller.

## 3. `verifier(world0) -> Verifier`

Built from the world **before**, called with a world **after**, returns a
`VerdictReport` with one of three verdicts and the numbers it was read off.

* `TRUE` needs evidence in the later world.
* `FALSE` is evidence of the opposite.
* `UNKNOWN` is what a verifier says when the robot cannot produce the evidence
  at all — an object nobody is detecting any more, a gripper with no report, a
  pour on a vessel with no scale under it.

Two rules hold for every verb, and both are tested:

* **Never TRUE by default — stated precisely.** A verifier handed the world it
  was built from, *in a world where its postcondition was not already true*,
  returns `FALSE` or `UNKNOWN`. The qualifier is load-bearing and was missing:
  most of these are **state** predicates, not progress predicates, and a state
  predicate handed a hand that is already holding the block is RIGHT to say
  `TRUE` — a `GoHome` verifier called on arms already at HOME likewise. What
  the rule forbids is a verdict that rests on the ACTION having been claimed
  rather than on the state having been measured. `tests/primitives/
  test_verifiers.py` therefore constructs each verb's "before" world with its
  postcondition deliberately unmet, and that is the shape the rule has.

  Two of them are genuinely **progress** predicates — `Nudge` and `Retreat`
  measure a displacement from the world they were built in — and for those an
  unchanged world can never pass, unconditionally.
* **Graded against what was asked.** A displacement verifier (`Nudge`,
  `Retreat`) allows `max(3 mm, 0.4 × |Δ|)`, not one fixed window: the old
  20 mm tolerance was wider than the 10 mm bottom of `NUDGE_GRID_M`, so the
  finest correction on the menu scored TRUE even when the hand had not moved
  at all. An *arrival* is still judged against a place (`TOOL_TOL_M`, 20 mm),
  because that is what an absolute pose is.
* **Measured, not intended.** `Grasp` reads `GripperReport.holding`, the
  torque-stop verdict off the wire, and cross-checks the jaw gap against the
  object's width so a gripper stalled on its own pads does not pass. `Place`
  wants the object inside the container's interior AABB *and* out of the jaws.

## Kinematic and learned verbs

Two kinds under one contract.

**Kinematic** verbs plan their own motion: `Approach`, `Grasp`, `Lift`,
`Carry`, `Place`, `Release`, `Nudge`, `Retreat`, `GoHome`.

**Learned** verbs subclass `LearnedPrimitive`. The kit still owns their
preconditions and their measured verifier; `plan()` returns
`PlanError("learned_policy_required")` naming the policy. That is the contract,
not a stub: a consumer holding an executor that can run the policy handles that
reason by running it, and one that cannot reports it as the reason the verb is
unavailable.

`Pour` is the first, and the reason the split exists (Shu, 2026-09-19:
「Pour は ACT」). Its executor lives in `d1-inference`, with the checkpoint: it
cannot live in the kit, which has no model runtime, and it cannot live in
`omakase-core`, which must not depend on `d1-inference`.

## Orientation is derived, never emitted

The model names a verb, an object and a `direction` — which way the hand
travels: an alias (`down`, `up`, `forward`, `backward`, `left`, `right`,
`along_tool`) or `{axis: [x, y, z], frame: base|tool|object:<name>}`
(`manipulation_kit.world.Direction`). The kit derives the wrist quaternion
from that direction plus the object's principal axis
(`primitives.orientation.align_tool`, the only place one is produced) — the
jaws close *across* the long side — and the per-arm mirror convention lives in
one constant. `Nudge` is the only verb that takes free numbers: translations
snapped to the ±10/30/50 mm grid, and a yaw clamped to ±15° about the approach
axis.

## Executors

`plan()` produces a `Plan`; an `Executor` runs one. The protocol
(`state` / `send_joints` / `set_gripper` / `settle`) lives in
`manipulation_kit.executor` with two pure test doubles. The only implementation
that opens a socket is `manipulation_kit.executors.firmware`, behind the
`[firmware]` extra — see its module docstring for why the default transport is
a daemon-played trajectory rather than a 50 Hz stream, and
`manipulation_kit.executors.firmware.ensure` for how its client is kept in step
with the daemon's own OpenAPI document.

---

## Addendum, 2026-09-19: what a verdict is allowed to rest on

Added after the review of PR #16. Three rules, each of which had a
counterexample in the code before it was written down.

**A verifier may not certify evidence that is not there.** A missing gripper
report is `UNKNOWN`, never "released" and never "still holding". An object that
is not in the later observation is `UNKNOWN`, never "it did not move". The
failure this prevents is a task marked done because nobody was looking; the
failure it must not cause is a working robot that looks broken, which is why
`UNKNOWN` is its own verdict and not a `FALSE`.

**A verifier may not certify identity it cannot see.** A torque stall proves
that *something* stopped the jaws. It does not distinguish the named block from
anything else of the same width, and a gripper reporting a *different*
`held_object` is a `FALSE`, not a detail. A named pickup becomes `TRUE` when
the producer names what it holds, or when the named object is measured at the
tool point; with neither, `UNKNOWN`, and the verdict names what would settle
it.

**A predicate is a set of measured clauses, and every clause is written out.**
"Placed" is: the object's own *extent* inside the destination, its underside on
the floor it should be standing on, the gripper released, and the gripper not
holding something else instead. Not its centre, and not "the gripper dict was
empty so it must have let go". Where the robot cannot measure a clause — this
one publishes no velocity, so "at rest" is not available — the verdict says
`supported` and says so explicitly rather than borrowing the stronger word.

### And what a PLAN is allowed to rest on

**Geometry is asked about the axis it happens on.** Jaw fit is the object's
extent along that grasp's jaw axis; support height is its extent along base
+z from its *resolved* orientation. `min(size)` is not conservative, it is a
different question. A tilt this version does not model is refused, not
approximated.

**A plan is bound to what it was checked against.** The measured joints of both
arms, the observation revision, the frame stamps and the tool geometry travel
with it, and an executor compares them before it moves. A plan is a safety
claim about one world; running it in another is running an unchecked path.

**A constrained leg does not take a detour.** Free-space transit may route
around a refusal; a grasp descent, a lift, a bounded nudge and a retreat may
not, because for those the shape of the path is the promise and an endpoint
that was reached the long way has not kept it.
