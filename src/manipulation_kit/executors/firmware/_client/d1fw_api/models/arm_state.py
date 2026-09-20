from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.arm_mode_type_0 import ArmModeType0
from ..models.arm_mode_type_1 import ArmModeType1
from ..models.arm_mode_type_2 import ArmModeType2
from ..models.arm_mode_type_3 import ArmModeType3
from ..models.arm_mode_type_4 import ArmModeType4
from ..models.arm_mode_type_5 import ArmModeType5

if TYPE_CHECKING:
    from ..models.arm_mode_type_6 import ArmModeType6


T = TypeVar("T", bound="ArmState")


@_attrs_define
class ArmState:
    """Arm feedback and command echo.

    Attributes:
        command_joints (list[float]): Seven arm joint values in SDK order J1 through J7, in degrees unless a
            field explicitly documents another unit.
        error_code (int): Controller error code, zero when no error is reported.
        feedback_joints (list[float]): Seven arm joint values in SDK order J1 through J7, in degrees unless a
            field explicitly documents another unit.
        feedback_temperature (list[float]): Seven arm joint values in SDK order J1 through J7, in degrees unless a
            field explicitly documents another unit.
        feedback_torque (list[float]): Seven arm joint values in SDK order J1 through J7, in degrees unless a
            field explicitly documents another unit.
        feedback_velocity (list[float]): Seven arm joint values in SDK order J1 through J7, in degrees unless a
            field explicitly documents another unit.
        frame_serial (int): Controller feedback frame serial.
        mode (ArmModeType0 | ArmModeType1 | ArmModeType2 | ArmModeType3 | ArmModeType4 | ArmModeType5 | ArmModeType6):
            The normalized arm feedback mode.
        stationary (bool): Whether the arm reports low-speed/stationary motion.
    """

    command_joints: list[float]
    error_code: int
    feedback_joints: list[float]
    feedback_temperature: list[float]
    feedback_torque: list[float]
    feedback_velocity: list[float]
    frame_serial: int
    mode: (
        ArmModeType0
        | ArmModeType1
        | ArmModeType2
        | ArmModeType3
        | ArmModeType4
        | ArmModeType5
        | ArmModeType6
    )
    stationary: bool
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        command_joints = self.command_joints

        error_code = self.error_code

        feedback_joints = self.feedback_joints

        feedback_temperature = self.feedback_temperature

        feedback_torque = self.feedback_torque

        feedback_velocity = self.feedback_velocity

        frame_serial = self.frame_serial

        mode: dict[str, Any] | str
        if (
            isinstance(self.mode, ArmModeType0)
            or isinstance(self.mode, ArmModeType1)
            or isinstance(self.mode, ArmModeType2)
            or isinstance(self.mode, ArmModeType3)
            or isinstance(self.mode, ArmModeType4)
            or isinstance(self.mode, ArmModeType5)
        ):
            mode = self.mode.value
        else:
            mode = self.mode.to_dict()

        stationary = self.stationary

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "command_joints": command_joints,
                "error_code": error_code,
                "feedback_joints": feedback_joints,
                "feedback_temperature": feedback_temperature,
                "feedback_torque": feedback_torque,
                "feedback_velocity": feedback_velocity,
                "frame_serial": frame_serial,
                "mode": mode,
                "stationary": stationary,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.arm_mode_type_6 import ArmModeType6

        d = dict(src_dict)
        command_joints = cast(list[float], d.pop("command_joints"))

        error_code = d.pop("error_code")

        feedback_joints = cast(list[float], d.pop("feedback_joints"))

        feedback_temperature = cast(list[float], d.pop("feedback_temperature"))

        feedback_torque = cast(list[float], d.pop("feedback_torque"))

        feedback_velocity = cast(list[float], d.pop("feedback_velocity"))

        frame_serial = d.pop("frame_serial")

        def _parse_mode(
            data: object,
        ) -> (
            ArmModeType0
            | ArmModeType1
            | ArmModeType2
            | ArmModeType3
            | ArmModeType4
            | ArmModeType5
            | ArmModeType6
        ):
            try:
                if not isinstance(data, str):
                    raise TypeError()
                componentsschemas_arm_mode_type_0 = ArmModeType0(data)

                return componentsschemas_arm_mode_type_0
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            try:
                if not isinstance(data, str):
                    raise TypeError()
                componentsschemas_arm_mode_type_1 = ArmModeType1(data)

                return componentsschemas_arm_mode_type_1
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            try:
                if not isinstance(data, str):
                    raise TypeError()
                componentsschemas_arm_mode_type_2 = ArmModeType2(data)

                return componentsschemas_arm_mode_type_2
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            try:
                if not isinstance(data, str):
                    raise TypeError()
                componentsschemas_arm_mode_type_3 = ArmModeType3(data)

                return componentsschemas_arm_mode_type_3
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            try:
                if not isinstance(data, str):
                    raise TypeError()
                componentsschemas_arm_mode_type_4 = ArmModeType4(data)

                return componentsschemas_arm_mode_type_4
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            try:
                if not isinstance(data, str):
                    raise TypeError()
                componentsschemas_arm_mode_type_5 = ArmModeType5(data)

                return componentsschemas_arm_mode_type_5
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            if not isinstance(data, dict):
                raise TypeError()
            componentsschemas_arm_mode_type_6 = ArmModeType6.from_dict(data)

            return componentsschemas_arm_mode_type_6

        mode = _parse_mode(d.pop("mode"))

        stationary = d.pop("stationary")

        arm_state = cls(
            command_joints=command_joints,
            error_code=error_code,
            feedback_joints=feedback_joints,
            feedback_temperature=feedback_temperature,
            feedback_torque=feedback_torque,
            feedback_velocity=feedback_velocity,
            frame_serial=frame_serial,
            mode=mode,
            stationary=stationary,
        )

        arm_state.additional_properties = d
        return arm_state

    @property
    def additional_keys(self) -> list[str]:
        return list(self.additional_properties.keys())

    def __getitem__(self, key: str) -> Any:
        return self.additional_properties[key]

    def __setitem__(self, key: str, value: Any) -> None:
        self.additional_properties[key] = value

    def __delitem__(self, key: str) -> None:
        del self.additional_properties[key]

    def __contains__(self, key: str) -> bool:
        return key in self.additional_properties
