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
from ..types import UNSET, Unset

if TYPE_CHECKING:
    from ..models.advisory import Advisory
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
        advisory (Advisory | None | Unset):
        brakes_released (bool | Unset): Whether this daemon has forced this arm's holding brakes OPEN for hand
            guiding (`POST /v1/arm/{side}/brake_release`) and not engaged them
            since. While `true` nothing holds the arm but whoever is supporting
            it; `GET /v1/arm/{side}/brake` has the window and who asked.

            Like [`Self::advisory`], this is the daemon's own record and not
            something the controller reports, so it is attached where the daemon
            hands an [`ArmState`] out and is `false` on a read taken straight from
            the device. It is NOT [`ArmMode::Release`], which is a controller mode.
        controller_version (int | None | Unset): The arm controller's firmware version, or `None` until it has been
            read.

            Reported identically on both sides because there is ONE controller
            behind both arms. It is read once at startup from the vendor's
            `VERSION` parameter — the same read the vendor's own SDK performs when
            it connects — and it is what says which host-to-controller wire
            generation the controller speaks.
        frame_miss_count (int | Unset): `RT_IN.m_FrameMissCnt`: how many host command frames the controller
            reckons it did not receive.

            The controller's OWN measure of host-side delivery jitter, refreshed
            every 200 control cycles, with the vendor's healthy band being below
            20. It is the field that decides whether a stuttering arm is this
            host's cadence or the controller's own scheduling — see
            [`Self::sys_cycle_miss_count`] for the other half of that split.
        max_frame_miss_count (int | Unset): The high-water mark of [`Self::frame_miss_count`] since the controller
            started. It never falls, so a zero here means an arm that has never
            stuttered rather than one that is not stuttering right now.
        protocol_mismatch (bool | Unset): Whether arm motion is currently REFUSED because the controller's
            firmware generation and the daemon's configured `[arm] protocol`
            disagree.

            The two host-to-controller generations differ by a leading `[i32
            serial]` in the joint-command record, and a controller on the other
            generation does not reject the frame — it parses it shifted by one
            element. Measured on d1-1 (controller `100343024`) on 2026-09-17: a
            position-mode move under the older framing moved every joint toward
            its NEIGHBOUR's target. So this is a refusal and not a warning, and it
            is published here because an operator whose commands are being refused
            needs to see why without reading the daemon's log.

            `false` while the version has not been read, and `false` when
            `[arm] allow_protocol_mismatch` is set for bench work.
        sys_cycle_miss_count (int | Unset): `RT_IN.m_SysCycMissCnt`: how many of the CONTROLLER's own realtime
            cycles missed their deadline.
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
    advisory: Advisory | None | Unset = UNSET
    brakes_released: bool | Unset = UNSET
    controller_version: int | None | Unset = UNSET
    frame_miss_count: int | Unset = UNSET
    max_frame_miss_count: int | Unset = UNSET
    protocol_mismatch: bool | Unset = UNSET
    sys_cycle_miss_count: int | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        from ..models.advisory import Advisory

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

        advisory: dict[str, Any] | None | Unset
        if isinstance(self.advisory, Unset):
            advisory = UNSET
        elif isinstance(self.advisory, Advisory):
            advisory = self.advisory.to_dict()
        else:
            advisory = self.advisory

        brakes_released = self.brakes_released

        controller_version: int | None | Unset
        if isinstance(self.controller_version, Unset):
            controller_version = UNSET
        else:
            controller_version = self.controller_version

        frame_miss_count = self.frame_miss_count

        max_frame_miss_count = self.max_frame_miss_count

        protocol_mismatch = self.protocol_mismatch

        sys_cycle_miss_count = self.sys_cycle_miss_count

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
        if advisory is not UNSET:
            field_dict["advisory"] = advisory
        if brakes_released is not UNSET:
            field_dict["brakes_released"] = brakes_released
        if controller_version is not UNSET:
            field_dict["controller_version"] = controller_version
        if frame_miss_count is not UNSET:
            field_dict["frame_miss_count"] = frame_miss_count
        if max_frame_miss_count is not UNSET:
            field_dict["max_frame_miss_count"] = max_frame_miss_count
        if protocol_mismatch is not UNSET:
            field_dict["protocol_mismatch"] = protocol_mismatch
        if sys_cycle_miss_count is not UNSET:
            field_dict["sys_cycle_miss_count"] = sys_cycle_miss_count

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.advisory import Advisory
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

        def _parse_advisory(data: object) -> Advisory | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            try:
                if not isinstance(data, dict):
                    raise TypeError()
                advisory_type_1 = Advisory.from_dict(data)

                return advisory_type_1
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            return cast(Advisory | None | Unset, data)

        advisory = _parse_advisory(d.pop("advisory", UNSET))

        brakes_released = d.pop("brakes_released", UNSET)

        def _parse_controller_version(data: object) -> int | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(int | None | Unset, data)

        controller_version = _parse_controller_version(
            d.pop("controller_version", UNSET)
        )

        frame_miss_count = d.pop("frame_miss_count", UNSET)

        max_frame_miss_count = d.pop("max_frame_miss_count", UNSET)

        protocol_mismatch = d.pop("protocol_mismatch", UNSET)

        sys_cycle_miss_count = d.pop("sys_cycle_miss_count", UNSET)

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
            advisory=advisory,
            brakes_released=brakes_released,
            controller_version=controller_version,
            frame_miss_count=frame_miss_count,
            max_frame_miss_count=max_frame_miss_count,
            protocol_mismatch=protocol_mismatch,
            sys_cycle_miss_count=sys_cycle_miss_count,
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
