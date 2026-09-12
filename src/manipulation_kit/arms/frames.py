"""Frame conventions between an input device and the arm's base frame.

Migrated from dx-vr-teleop ``server/clutch.py`` (post-#41 ``master``), where the
same matrix is used, and independently restated in omakase-core #94
``robot_stack/teleop/vr_arm/clutch.py``. It lives here because BOTH consumers
of the arm are VR-driven: dx-vr-teleop's WebXR server and the runtime that
starts VR teleop mid-conversation. It is pure convention, so it belongs in the
one place both can import.

WebXR headset frame: x right, y up, z BACKWARD (toward the viewer).
Robot base frame:    x forward, y left, z up.

    robot_x = -webxr_z,  robot_y = -webxr_x,  robot_z = webxr_y
"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as R

#: WebXR -> robot base rotation (x_r = -z_w, y_r = -x_w, z_r = y_w)
WEBXR2ROBOT = np.array([[0.0, 0.0, -1.0],
                        [-1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0]])


def webxr_to_robot(p_w) -> np.ndarray:
    """A WebXR-frame position re-expressed in the robot base frame."""
    return WEBXR2ROBOT @ np.asarray(p_w, dtype=float)


def webxr_quat_to_robot(q_xyzw) -> R:
    """A WebXR-frame orientation (quaternion xyzw) in the robot base frame."""
    r = R.from_quat(np.asarray(q_xyzw, dtype=float))
    m = WEBXR2ROBOT @ r.as_matrix() @ WEBXR2ROBOT.T
    return R.from_matrix(m)
