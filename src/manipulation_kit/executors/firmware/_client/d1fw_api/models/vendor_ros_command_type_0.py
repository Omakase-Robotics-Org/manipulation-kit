from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar

from attrs import define as _attrs_define
from typing_extensions import Self

from ..models.vendor_ros_command_type_0_command import VendorRosCommandType0Command

if TYPE_CHECKING:
    from ..models.vendor_mapping_expected import VendorMappingExpected


T = TypeVar("T", bound="VendorRosCommandType0")


@_attrs_define
class VendorRosCommandType0:
    """Publish one fixed /reset_pose geometry_msgs/PoseStamped localization initial pose.

    Read /v1/chassis/ros/state and /v1/chassis/ros/capabilities before review.
    Requires configured/available bridge, commands_enabled=true, connected current-session
    /monitor_state and /chassis_state readings with status=fresh and no topic error.
    Receipt age must be below stale_after_ms (currently 3000 ms); this is not sensor time.
    Monitor must report work_mode=1 (AUTO_NAV), auto_status=2 (READY), dock_status=0,
    and the exact reviewed current_scene in its complete all-string scenes inventory.
    Chassis wheel_speed must be numeric zero; emergence_status, strip_status,
    standby_status, cpole_status, l_motor_error_code, r_motor_error_code and encoder_error
    must all be known zero. Active/paused vendor navigation states are not admitted.
    Software kill, control-epoch changes and conflicting chassis/jog transitions refuse
    admission. Freshness/interlocks and reviewed state are checked again before publishing.
    READY does not prove pose correctness. This does not wake, navigate, change mode,
    or perform HTTP Recover.

    Fixed frame map and z=0.085 m; quaternion is (0,0,sin(yaw_rad/2),cos(yaw_rad/2)).
    The fixed z is a message constant, not measured height. Map origin, physical axis
    calibration and robot reference point are not established by this API; callers must
    use independently reviewed scene coordinates, not infer them from READY.

    One publish attempt after fixed-type advertisement; no subscriber acknowledgment.
    Only transport_sent or outcome_unknown reports are valid, with vendor_success=null
    and physical_completion=unknown. Reread and review before another explicit request;
    never retry automatically. A matching /curr_pose observation is not causal proof.

        Attributes:
            command (VendorRosCommandType0Command):
            expected (VendorMappingExpected): Operator-observed mode precondition, checked again immediately before sending.
            scene (str): Exact current existing scene: nonblank, at most 256 UTF-8 bytes, no Unicode
                control characters. Must equal expected.current_scene and current monitor scene,
                and occur in the complete reported scenes list. No trimming or normalization.
            x_m (float): Map-frame x coordinate in metres.
            y_m (float): Map-frame y coordinate in metres.
            yaw_rad (float): Map-frame yaw in radians, within [-pi, pi].
    """

    command: VendorRosCommandType0Command
    expected: VendorMappingExpected
    scene: str
    x_m: float
    y_m: float
    yaw_rad: float

    def to_dict(self) -> dict[str, Any]:
        command = self.command.value

        expected = self.expected.to_dict()

        scene = self.scene

        x_m = self.x_m

        y_m = self.y_m

        yaw_rad = self.yaw_rad

        field_dict: dict[str, Any] = {}

        field_dict.update(
            {
                "command": command,
                "expected": expected,
                "scene": scene,
                "x_m": x_m,
                "y_m": y_m,
                "yaw_rad": yaw_rad,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.vendor_mapping_expected import (
            VendorMappingExpected,
        )

        d = dict(src_dict)
        command = VendorRosCommandType0Command(d.pop("command"))

        expected = VendorMappingExpected.from_dict(d.pop("expected"))

        scene = d.pop("scene")

        x_m = d.pop("x_m")

        y_m = d.pop("y_m")

        yaw_rad = d.pop("yaw_rad")

        vendor_ros_command_type_0 = cls(
            command=command,
            expected=expected,
            scene=scene,
            x_m=x_m,
            y_m=y_m,
            yaw_rad=yaw_rad,
        )

        return vendor_ros_command_type_0
