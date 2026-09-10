"""Relative-follow EE targeting with hard safety clamps (pure math, testable).

Migrated from dx-vr-teleop ``server/clutch.py`` (post-#41 ``master``); the same
logic was independently restated in omakase-core #94
``robot_stack/teleop/vr_arm/clutch.py``. This module is the third copy's
replacement, not a fourth: the arithmetic is unchanged.

Semantics: while an arm is ENGAGED the arm follows the input device RELATIVELY —

    T_target = T_ee_anchor . D,   D = map(T_ctrl_now (-) T_ctrl_anchor)

where both anchors are captured at the moment of engagement (re-anchoring — the
controller can be anywhere in the room; only motion after engaging counts, the
proven yubi-vr-teleop clutch UX).

Signal path, applied in order, every tick:

  0. One-Euro pose filter — the raw controller pose carries 8-12 Hz hand tremor
     plus tracking noise; unfiltered it makes the wrist visibly tremble during
     SLOW moves (noise dominates the signal). Reset at every engage;
  1. workspace box clamp — target position clipped into the belly-front box.
     The bounds start as box union anchor (the anchor may be outside the box,
     e.g. engaging at HOME) and latch to the strict box once the target enters
     it, so engaging NEVER teleports the arm toward the box edge;
  2. per-tick step clamp — translation <= ``max_step_m``, rotation <=
     ``max_step_rad`` (a controller glitch — or the box clamp itself — cannot
     fling the arm);
  3. parking deadband, WITH HYSTERESIS — a target within the deadband of the
     last ACCEPTED target is not re-issued (returns ``None``): a stationary
     hand stops producing fresh IK solves, so the solver cannot dither the
     joints inside its convergence tolerance while "holding still". Leaving
     the deadband costs ``park_release_mult`` x entering it, so the tail of
     the filter's convergence after the hand stops cannot step back and forth
     across a single edge (the twitches at the END of a move);
  4. (caller) IK + collision guard — a rejected solve HOLDS the last valid
     target: the arm stops at the body boundary and never passes through.

WHAT THIS MODULE IS NOT: it has no idea what engaged it. Grip-button decoding,
lock/session semantics, who currently owns the arm, and anything with a socket
stay with the consumer. This is a per-arm state machine over poses, which is
why both dx-vr-teleop's server and omakase-core's runtime can drive it.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R

from . import safety
from .filters import PoseFilter
from .frames import webxr_quat_to_robot, webxr_to_robot


@dataclass(frozen=True)
class ClutchTuning:
    """The clamps :class:`ArmClutch` enforces.

    ``workspace`` has NO default on purpose. It is a measured, per-robot
    geometric envelope, and a safety box that silently defaults to some other
    robot's numbers is the failure mode this whole package exists to remove —
    ask the arm for it (:func:`manipulation_kit.arms.get_clutch_tuning`). The rate and
    deadband terms are tick-rate/noise derived rather than geometry derived, so
    they do carry the shared defaults from :mod:`manipulation_kit.arms.safety`.
    """

    workspace: Dict[str, Tuple[float, float]]
    pos_scale: float = safety.POS_SCALE
    max_step_m: float = safety.MAX_STEP_M
    max_step_rad: float = safety.MAX_STEP_RAD
    park_pos_m: float = safety.PARK_POS_M
    park_rot_rad: float = safety.PARK_ROT_RAD
    park_release_mult: float = safety.PARK_RELEASE_MULT

    def bounds(self) -> List[Tuple[float, float]]:
        """The box as an ordered ``[(lo, hi)] * 3`` list in x, y, z."""
        return [self.workspace["x"], self.workspace["y"], self.workspace["z"]]


@dataclass
class ArmClutch:
    """One arm's relative-follow state."""

    tuning: ClutchTuning
    engaged: bool = False
    _ctrl_p0: Optional[np.ndarray] = None
    _ctrl_r0: Optional[R] = None
    _ee_p0: Optional[np.ndarray] = None
    _ee_r0: Optional[R] = None
    #: last SAFE target actually accepted (IK + guard passed) — the hold pose.
    last_target_p: Optional[np.ndarray] = None
    last_target_r: Optional[R] = None
    rejected_streak: int = field(default=0)
    #: True while the hand counts as PARKED. It is a STATE and not a fresh
    #: per-tick comparison because leaving the deadband costs
    #: ``park_release_mult`` x entering it — see target() step 3.
    _parked: bool = False
    #: workspace bounds in effect: box union anchor until the target first
    #: enters the strict box, then latched to the strict box (see engage()).
    _bounds: List[Tuple[float, float]] = field(default_factory=list)
    _latched: bool = False
    #: One-Euro filter over the raw controller pose (tremor/tracking noise).
    #: filter_enabled=False bypasses it (exact-math tests; NOT for operation).
    _filter: PoseFilter = field(default_factory=PoseFilter)
    filter_enabled: bool = True
    #: Strapped-controller mount rotation (robot frame). When the controller is
    #: strapped to the forearm at an arbitrary offset, the rotation DELTA is
    #: conjugated into the hand frame:
    #:     D_hand = R_mount^-1 . D_ctrl . R_mount
    #: i.e. the delta's rotation AXIS is re-expressed through the mount offset
    #: while the angle is preserved. Captured one-shot via a calibration chord
    #: by the consumer and persisted there; None = identity (handheld).
    #: NOTE: omakase-core #94 does not have this — it is one of the features a
    #: shared implementation hands it for free.
    mount_rot: Optional[R] = None

    # ------------------------------------------------------------------ #
    def engage(self, ctrl_p_w, ctrl_q_w, ee_p, ee_r: R) -> None:
        """Anchor at the CURRENT controller + EE pose (called on engage)."""
        self.engaged = True
        # a fresh engage is never parked: the anchor IS the new hold pose, so
        # the first real motion must be issued against the entry threshold and
        # not against a release threshold left over from the last session
        self._parked = False
        # fresh filter state per engage: stale smoothing from a previous
        # engagement (possibly minutes old, elsewhere in the room) must not
        # bleed into the new anchor as a phantom offset
        self._filter.reset()
        self._ctrl_p0 = webxr_to_robot(ctrl_p_w)
        self._ctrl_r0 = webxr_quat_to_robot(ctrl_q_w)
        self._ee_p0 = np.asarray(ee_p, dtype=float).copy()
        self._ee_r0 = ee_r
        self.last_target_p = self._ee_p0.copy()
        self.last_target_r = ee_r
        # The anchor may sit OUTSIDE the workspace box (e.g. engaging at HOME):
        # clamping straight to the box would teleport the arm the instant it
        # engages, with the operator's hand perfectly still. Start with
        # per-axis bounds expanded to include the anchor and LATCH to the
        # strict box once the target has entered it.
        lo_hi = self.tuning.bounds()
        self._bounds = [(min(lo, p), max(hi, p))
                        for (lo, hi), p in zip(lo_hi, self._ee_p0)]
        self._latched = all(lo <= p <= hi
                            for (lo, hi), p in zip(lo_hi, self._ee_p0))
        if self._latched:
            self._bounds = lo_hi

    def disengage(self) -> None:
        self.engaged = False
        self._parked = False

    # ------------------------------------------------------------------ #
    def target(self, ctrl_p_w, ctrl_q_w, t: Optional[float] = None
               ) -> Optional[Tuple[np.ndarray, R]]:
        """The clamped EE target for this controller frame.

        Returns ``None`` when disengaged OR when the (filtered) hand is parked,
        so a stationary hand issues no fresh IK solves (anti-jitter hold).
        Parking is HYSTERETIC: entered within the deadband of the last accepted
        target, left only past ``park_release_mult`` x that deadband — step 3.
        """
        if not self.engaged or self._ctrl_p0 is None:
            return None
        if t is None:
            import time
            t = time.monotonic()
        tun = self.tuning
        strict = tun.bounds()
        # 0) One-Euro smoothing of the RAW controller pose (webxr frame; the
        #    frame change below is linear, so filtering before/after is
        #    equivalent — before keeps the filter unit-consistent). Note
        #    pos_scale amplifies controller noise too, which is part of why
        #    unfiltered slow motion visibly trembled at the wrist.
        if self.filter_enabled:
            p_ctrl, q_ctrl = self._filter(ctrl_p_w, ctrl_q_w, t)
        else:
            p_ctrl, q_ctrl = ctrl_p_w, ctrl_q_w
        dp = webxr_to_robot(p_ctrl) - self._ctrl_p0
        p_raw = self._ee_p0 + tun.pos_scale * dp
        r_now = webxr_quat_to_robot(q_ctrl)
        r_delta = r_now * self._ctrl_r0.inv()
        if self.mount_rot is not None:
            # strapped controller: conjugate the delta through the captured
            # mount offset so it is expressed as the HAND's rotation, not the
            # controller's:  D_hand = R_mount^-1 D_ctrl R_mount
            r_delta = self.mount_rot.inv() * r_delta * self.mount_rot
        r_raw = r_delta * self._ee_r0

        # 1) workspace box clamp FIRST, so any clamp-induced correction is
        #    still step-limited below (clamping after the step clamp could
        #    move the target arbitrarily far in a single tick).
        p_raw = np.array([np.clip(v, lo, hi)
                          for v, (lo, hi) in zip(p_raw, self._bounds)])
        if not self._latched and all(
                lo <= v <= hi for v, (lo, hi) in zip(p_raw, strict)):
            self._latched = True
            self._bounds = strict

        # 2) per-tick step clamp vs the LAST accepted target
        p_ref = self.last_target_p if self.last_target_p is not None else self._ee_p0
        step = p_raw - p_ref
        n = float(np.linalg.norm(step))
        if n > tun.max_step_m:
            p_raw = p_ref + step * (tun.max_step_m / n)
        r_ref = self.last_target_r if self.last_target_r is not None else self._ee_r0
        dr = r_raw * r_ref.inv()
        ang = float(np.linalg.norm(dr.as_rotvec()))
        if ang > tun.max_step_rad:
            r_raw = R.from_rotvec(dr.as_rotvec() * (tun.max_step_rad / ang)) * r_ref

        # 3) parking deadband, WITH HYSTERESIS: a stationary hand must not keep
        #    re-issuing near-identical targets — every re-solve lets the IK
        #    dither the joints inside its convergence tolerance (the
        #    slow-motion tremble).
        #
        #    The deadband has TWO edges. Entering costs park_pos_m /
        #    park_rot_rad; leaving costs park_release_mult x that. One edge sits
        #    exactly where a stopping hand comes to rest: the One-Euro filter
        #    above runs its lowest cutoff at low speed, so it goes on converging
        #    for several ticks after the hand has stopped, and the tail of that
        #    convergence arrives here as a few discrete steps just OVER
        #    park_pos_m. Every one of them re-solves IK and the joints twitch
        #    inside its tolerance — the 2-3 small movements at the END of a
        #    move — while a hand held on the edge toggles park -> release ->
        #    park every other tick. Two edges swallow both: the deviation has to
        #    be real motion before the arm is asked to move again. See
        #    safety.PARK_RELEASE_MULT for why the factor is 2.
        #
        #    Only park when the last tick was ACCEPTED: after a reject the state
        #    near the boundary should keep re-trying, not freeze — which also
        #    means a rejected tick LEAVES the parked state rather than holding
        #    it across the retries.
        if self.rejected_streak or self.last_target_p is None:
            self._parked = False
        else:
            d_pos = float(np.linalg.norm(p_raw - self.last_target_p))
            d_rot = float(np.linalg.norm(
                (r_raw * self.last_target_r.inv()).as_rotvec()))
            if self._parked:
                mult = tun.park_release_mult
                if (d_pos <= tun.park_pos_m * mult
                        and d_rot <= tun.park_rot_rad * mult):
                    return None
                self._parked = False
            elif d_pos < tun.park_pos_m and d_rot < tun.park_rot_rad:
                self._parked = True
                return None
        return p_raw, r_raw

    # ------------------------------------------------------------------ #
    def accept(self, p: np.ndarray, r: R) -> None:
        """IK + guard PASSED for (p, r) — it becomes the new hold pose."""
        self.last_target_p = np.asarray(p, dtype=float).copy()
        self.last_target_r = r
        self.rejected_streak = 0

    def reject(self) -> Optional[Tuple[np.ndarray, R]]:
        """IK + guard REJECTED — hold the last safe pose (boundary stop)."""
        self.rejected_streak += 1
        if self.last_target_p is None:
            return None
        return self.last_target_p, self.last_target_r
