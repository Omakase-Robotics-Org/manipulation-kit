"""One-Euro pose / joint filtering for arm following.

Migrated from dx-vr-teleop ``server/filters.py`` (post-#41 ``master``). The
algorithm is unchanged, character for character in its arithmetic — the only
edits are that the tuning constants now come from :mod:`manipulation_kit.arms.safety`
(one source of truth, see that module) and that the default cutoffs are
constructor arguments so a consumer can filter two things at different rates
without touching the environment.

See :mod:`manipulation_kit.arms.safety` for the tuning rationale (tremor band, why the
cutoff must adapt to speed, and the field report that prompted it).
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np

from . import safety


def _alpha(cutoff_hz: float, dt: float) -> float:
    tau = 1.0 / (2.0 * math.pi * cutoff_hz)
    return 1.0 / (1.0 + tau / dt)


class OneEuro:
    """Vector One-Euro filter (independent per component, shared speed)."""

    def __init__(self, min_cutoff: float, beta: float,
                 d_cutoff: float = safety.D_CUTOFF):
        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.d_cutoff = float(d_cutoff)
        self._x: Optional[np.ndarray] = None
        self._dx: Optional[np.ndarray] = None
        self._t: Optional[float] = None

    def reset(self) -> None:
        self._x = self._dx = self._t = None

    def __call__(self, x, t: float) -> np.ndarray:
        x = np.asarray(x, dtype=float)
        if self._x is None or self._t is None or t <= self._t:
            # first sample (or non-monotonic clock): pass through, seed state
            self._x, self._dx, self._t = x.copy(), np.zeros_like(x), float(t)
            return x.copy()
        dt = float(t) - self._t
        self._t = float(t)
        # derivative estimate, low-passed
        dx = (x - self._x) / dt
        a_d = _alpha(self.d_cutoff, dt)
        self._dx = self._dx + a_d * (dx - self._dx)
        speed = float(np.linalg.norm(self._dx))
        # speed-adaptive cutoff -> signal low-pass
        cutoff = self.min_cutoff + self.beta * speed
        a = _alpha(cutoff, dt)
        self._x = self._x + a * (x - self._x)
        return self._x.copy()


class PoseFilter:
    """One-Euro on position (m) + quaternion (xyzw) with sign continuity."""

    def __init__(self, pos_min_cutoff: float = safety.POS_MIN_CUTOFF,
                 pos_beta: float = safety.POS_BETA,
                 rot_min_cutoff: float = safety.ROT_MIN_CUTOFF,
                 rot_beta: float = safety.ROT_BETA):
        self._fp = OneEuro(pos_min_cutoff, pos_beta)
        self._fq = OneEuro(rot_min_cutoff, rot_beta)
        self._last_q: Optional[np.ndarray] = None

    def reset(self) -> None:
        self._fp.reset()
        self._fq.reset()
        self._last_q = None

    def __call__(self, p, q_xyzw, t: float) -> Tuple[np.ndarray, np.ndarray]:
        p_f = self._fp(p, t)
        q = np.asarray(q_xyzw, dtype=float)
        # sign continuity: q and -q are the same rotation; a raw sign flip
        # would look like a huge jump to the low-pass and glitch the output
        if self._last_q is not None and float(np.dot(q, self._last_q)) < 0.0:
            q = -q
        self._last_q = q.copy()
        q_f = self._fq(q, t)
        n = float(np.linalg.norm(q_f))
        q_f = q_f / n if n > 1e-9 else q
        return p_f, q_f


class JointFilter:
    """One-Euro filter for a single arm's joint vector (radians).

    Adapts smoothing to motion speed: strong smoothing when joints are nearly
    still (suppresses IK jitter), near-zero lag when the arm moves fast.
    """

    def __init__(self, min_cutoff: float = safety.JOINT_MIN_CUTOFF,
                 beta: float = safety.JOINT_BETA):
        self._f = OneEuro(min_cutoff, beta)

    def reset(self) -> None:
        self._f.reset()

    def __call__(self, q: np.ndarray, t: float) -> np.ndarray:
        return self._f(q, t)
