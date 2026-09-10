"""The calibration tool duplicates geometry facts so it can run on a bare
robot python (importing the package pulls in the CAN driver chain). These
tests are the anti-drift guard for that duplication, plus the solver's own
verifier."""
import importlib.util
import math
import os

_TOOL = os.path.join(os.path.dirname(__file__), "..", "tools",
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
