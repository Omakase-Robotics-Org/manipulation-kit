"""The calibration tool duplicates geometry facts so it can run on a bare robot
python. These tests are the anti-drift guard for that duplication, plus the
solver's own verifier.

The original reason for the duplication — "importing the package pulls in the
CAN driver chain" — is GONE: the drivers went to d1-firmwared and importing
``manipulation_kit.hands.d1.parallel_gripper`` now costs nothing but stdlib.
The tool is still loaded BY PATH here because it is a standalone script under
``tools/``, not an importable module, and because the duplication it guards is
still there. Collapsing the duplication is a separate, deliberate change."""
import importlib.util
import math
import os

import manipulation_kit

_TOOL = os.path.join(os.path.dirname(os.path.abspath(manipulation_kit.__file__)),
                     "hands", "d1", "parallel_gripper", "tools",
                     "calibrate_wrist_camera_mount.py")
_spec = importlib.util.spec_from_file_location("calibrate_wrist_camera_mount",
                                               _TOOL)
tool = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(tool)


def test_duplicated_constants_match_the_description():
    from manipulation_kit.hands.d1.parallel_gripper import description as desc

    assert tool.JAW_STROKE_M == desc.JAW_STROKE_M
    assert tool.JAW_TIP_Z_M == desc.JAW_TIP_Z_M
    assert tuple(tool.NOMINAL_XYZ) == desc.CAMERA_MOUNT_XYZ_M
    assert math.isclose(tool.NOMINAL_TILT_RAD, math.radians(15.0))


def test_selftest_recovers_an_injected_miscalibration():
    # 2 deg / 4 mm injected, 0.5 px annotation noise; the tool asserts
    # recovery to < 1 mm / < 0.2 deg internally.
    assert tool.selftest()
