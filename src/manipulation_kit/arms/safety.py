"""The arm's clamp / IK / workspace constants — ONE source of truth.

Every number here was measured or tuned on a real D1 and every one of them is
a SAFETY limit or the thing that keeps teleop from juddering. They are
collected in one module, with their provenance, because they were previously
restated in two places (dx-vr-teleop ``server/clutch.py`` + ``server/backends.py``
and omakase-core ``robot_stack/teleop/vr_arm/safety.py``) and the copies had
already begun to drift — see ``MAX_JOINT_STEP_RAD`` below for a live example.

A duplicated safety limiter is strictly worse than one shared limiter, because
when the two disagree nobody can say which one the robot is actually running.

Values are the dx-vr-teleop post-#41 values, which are the ones that have been
driven on hardware.

THE REFERENCE IMPLEMENTATION IS ``dx-vr-teleop`` ``master``. Where this library
and another consumer disagree about a value or a behaviour, master wins — it is
the code that runs on the robot. A consumer's differing number is a divergence to
be corrected, not an alternative to be reconciled. (Ruling by Shu, 2026-07-29, on
``MAX_JOINT_STEP_RAD``; stated generally because it settles the next one too.)
"""

from __future__ import annotations

import os
from typing import Dict, Tuple


def envf(name: str, default: float, *, legacy: str = "") -> float:
    """Read a float override from the environment.

    ``OMAKASE_ARM_*`` is the name this package owns. ``legacy`` names the
    ``D1_TELEOP_*`` variable dx-vr-teleop already reads, and is honoured as a
    fallback so an operator's existing tuning keeps working — and, crucially,
    so both copies of the code respond to the SAME environment while the
    duplication lasts. Setting only the legacy name in one process and the new
    name in another is exactly the divergence this module exists to prevent.
    """
    raw = os.environ.get(name)
    if raw is None and legacy:
        raw = os.environ.get(legacy)
    return default if raw is None else float(raw)


# --------------------------------------------------------------------------- #
# End-effector target shaping (per teleop tick, nominally 50 Hz)
# --------------------------------------------------------------------------- #

#: max EE translation per tick [m] — 50 Hz => a 1.5 m/s ceiling. A controller
#: glitch, a tracking dropout, or the workspace clamp itself must not be able
#: to fling the arm across its envelope in one tick.
MAX_STEP_M = 0.03

#: max EE rotation per tick [rad] — 50 Hz => ~344 deg/s ceiling.
MAX_STEP_RAD = 0.12

#: Parking deadband vs the last ACCEPTED target. Below this the hand counts as
#: stationary and no new target is issued, so a still hand stops producing
#: fresh IK solves — otherwise the solver dithers the joints inside its own
#: convergence tolerance and the wrist visibly trembles while "holding still"
#: (プルプル, reported by the Japan team 2026-07-13). Big enough to swallow
#: residual filtered noise, far below any intentional motion.
PARK_POS_M = envf("OMAKASE_ARM_PARK_POS_M", 0.0025, legacy="D1_TELEOP_PARK_POS_M")
PARK_ROT_RAD = envf("OMAKASE_ARM_PARK_ROT_RAD", 0.012, legacy="D1_TELEOP_PARK_ROT_RAD")

#: Hysteresis on LEAVING the parking deadband, as a MULTIPLE of the two
#: thresholds above: entering costs PARK_POS_M / PARK_ROT_RAD, leaving costs
#: this many times that.
#:
#: A one-edged deadband puts its edge exactly where a stopping hand comes to
#: rest. The One-Euro filter above runs a LOW cutoff at low speed, so it keeps
#: converging for several ticks after the hand itself has stopped, and the tail
#: of that convergence reaches the clutch as a handful of discrete steps just
#: over PARK_POS_M. Each one is a fresh IK solve, each solve may dither the
#: joints inside IK_POS_TOL, and the operator sees the arm twitch two or three
#: times AFTER they stopped moving (クイクイ, reported by the Japan team
#: 2026-08-16 — the same mechanism as the slow-motion プルプル PARK_POS_M was
#: added for, one deadband edge further out). A hand held ON the edge is the
#: other half: it toggles park -> release -> park every other tick.
#:
#: 2x is chosen to be larger than both of those (the residual tail and hand
#: micro-tremor are what PARK_POS_M itself was sized to swallow, so twice it
#: clears them with margin) while staying far below any intentional motion:
#: 5 mm / 0.024 rad is still a small fraction of one MAX_STEP_M tick. Setting
#: it to 1.0 disables the hysteresis and restores the single-edged behaviour.
PARK_RELEASE_MULT = envf("OMAKASE_ARM_PARK_RELEASE_MULT", 2.0)

#: Translation gain: the EE moves POS_SCALE x the controller displacement, so a
#: modest hand reach drives the arm out to its (kinematically short, ~0.5 m
#: forward) envelope without the operator fully extending. Rotation stays 1:1.
#: 1.0 = literal hardware-teleop feel. IK + the collision guard remain the true
#: limiters.
POS_SCALE = 1.5

#: Workspace box [m, robot base frame]. A COARSE SUPERSET of the reachable
#: space, NOT its shape — IK + guard are the true limiters (an unreachable
#: target just holds the last safe pose). The reachable set is not a box:
#: lateral reach bulges to |y| ~ 0.70 when the hand is beside the body (x ~ 0)
#: but collapses to ~0.55 pushed forward (x ~ 0.35), so a horizontal T-stretch
#: needs BOTH a low x-floor and a wide y. An earlier tight box (x >= 0.15,
#: |y| <= 0.55) clipped the reachable lateral bulge -> "the arm won't follow me
#: sideways". NOTE the x-floor is 0.15, NOT 0: letting the hand reach x < 0.15
#: near the midline commands it INTO the torso keep-out, the guard then rejects
#: every such tick and the arm holds/jerks ("weird IK"). The lateral bulge is
#: nearly flat there (ymax 0.675 at x=0.15 vs 0.70 at x=0), so almost all the
#: sideways reach survives while staying out of the body.
WORKSPACE: Dict[str, Tuple[float, float]] = {
    "x": (0.15, 0.55),
    "y": (-0.72, 0.72),
    "z": (0.05, 0.85),
}

# --------------------------------------------------------------------------- #
# Joint-space clamps
# --------------------------------------------------------------------------- #

#: Max per-joint change per ACCEPTED tick [rad]. Teleop following moves a few
#: hundredths of a rad per joint per tick; an IK branch flip is >= 0.5, so this
#: is the guard against a discontinuous re-configuration reaching the arm.
#:
#: RESOLVED 2026-07-29 (Shu): 0.25 is correct. dx-vr-teleop ran 0.25
#: (``backends.MAX_JOINT_STEP``, hardware-driven) while omakase-core #94's
#: ``safety.py`` independently chose 0.05 — a 5x difference in a safety clamp,
#: arrived at because the value was copied rather than shared. That 0.05 was a
#: DIVERGENCE, not a competing opinion: dx-vr-teleop ``master`` is the reference
#: implementation for shared-library behaviour, and 0.25 is confirmed against it.
#: Kept as history because the episode is the whole argument for this package
#: existing.
#:
#: The over-solve is executed PARTIALLY (scaled along q0 -> q) rather than
#: rejected, so a lower value costs tracking speed instead of freezing.
MAX_JOINT_STEP_RAD = 0.25

# --------------------------------------------------------------------------- #
# IK solver
# --------------------------------------------------------------------------- #

#: damped-least-squares iteration cap per solve
IK_ITERS = 60
#: position convergence tolerance [m]
IK_POS_TOL = 2e-3
#: rotation convergence tolerance [rad]
IK_ROT_TOL = 5e-2
#: Levenberg damping added to J J^T
IK_DAMPING = 1e-4
#: null-space posture-pull gain toward the READY branch
IK_POSTURE_GAIN = 0.15
#: per-iteration cap on the posture pull [rad] — the elbow migrates toward
#: READY over many ticks instead of re-configuring inside one solve
IK_PULL_CLIP = 0.1
#: per-iteration cap on the joint update [rad]
IK_DQ_CLIP = 0.2

# --------------------------------------------------------------------------- #
# READY-posture seed search (one-time, at model load)
# --------------------------------------------------------------------------- #

#: random restarts
READY_SEED_SAMPLES = 200
#: deterministic RNG seed — the search MUST be reproducible, because the
#: posture it finds is the null-space bias every later solve drifts toward
READY_SEED_RNG = 0
#: iteration cap for the probe solves inside the search
READY_SEED_IK_ITERS = 50
#: probe target [m] in the robot base frame; y is mirrored per side
READY_SEED_TARGET_X = 0.30
READY_SEED_TARGET_Y = 0.18
READY_SEED_TARGET_Z = 0.30
#: forward-jut penalty weight. Pure max-clearance picked an elbow-up-and-
#: FORWARD branch for the right arm (+0.26 m fwd) that looked wrong; this
#: steers to an elbow-down-and-out branch of equal/better clearance without
#: jutting. An outboard/armpit penalty was TRIED AND REVERTED: tucking the
#: elbow in halved clearance (0.058 -> 0.031 on the right) and destabilised
#: teleop.
READY_SEED_FWD_W = envf("OMAKASE_ARM_SEED_FWD_W", 0.6, legacy="D1_SEED_FWD_W")
#: forward distance beyond which the penalty starts [m]
READY_SEED_FWD_FREE = 0.10

# --------------------------------------------------------------------------- #
# One-Euro filtering (Casiez et al., CHI 2012)
# --------------------------------------------------------------------------- #
# The Quest streams the raw controller pose at 50 Hz. Physiological hand tremor
# (8-12 Hz) plus tracking noise ride on top, and with no filter every
# sub-millimetre wiggle becomes a fresh IK target. During fast motion the noise
# is buried in the signal; during SLOW motion it dominates. One-Euro is the
# standard fix: an adaptive low-pass whose cutoff rises with speed — slow hand
# -> low cutoff -> tremor removed; fast hand -> high cutoff -> near-zero added
# latency. MIN_CUTOFF sits well below the tremor band but above intentional
# slow-move frequencies (~1 Hz); BETA sets how fast the filter lets go as the
# hand speeds up (units 1/m for position, 1/rad for rotation).

POS_MIN_CUTOFF = envf("OMAKASE_ARM_POS_MIN_CUTOFF", 0.5, legacy="D1_TELEOP_POS_MIN_CUTOFF")
POS_BETA = envf("OMAKASE_ARM_POS_BETA", 3.0, legacy="D1_TELEOP_POS_BETA")
ROT_MIN_CUTOFF = envf("OMAKASE_ARM_ROT_MIN_CUTOFF", 1.0, legacy="D1_TELEOP_ROT_MIN_CUTOFF")
ROT_BETA = envf("OMAKASE_ARM_ROT_BETA", 1.5, legacy="D1_TELEOP_ROT_BETA")
JOINT_MIN_CUTOFF = envf("OMAKASE_ARM_JOINT_MIN_CUTOFF", 1.0,
                        legacy="D1_TELEOP_JOINT_MIN_CUTOFF")
JOINT_BETA = envf("OMAKASE_ARM_JOINT_BETA", 10.0, legacy="D1_TELEOP_JOINT_BETA")
#: One-Euro internal derivative low-pass cutoff [Hz] — rarely tuned
D_CUTOFF = 1.0
