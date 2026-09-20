from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.work_mode_type_0 import WorkModeType0
from ..models.work_mode_type_1 import WorkModeType1
from ..models.work_mode_type_2 import WorkModeType2
from ..models.work_mode_type_3 import WorkModeType3
from ..models.work_mode_type_4 import WorkModeType4

if TYPE_CHECKING:
    from ..models.work_mode_type_5 import WorkModeType5


T = TypeVar("T", bound="ChassisState")


@_attrs_define
class ChassisState:
    """Chassis feedback, including whether the mobile base is there at all.

    A D1 is regularly run with its vendor chassis powered off — on external
    power, using only the lift, arms, torso and head. The chassis controller is
    then a dead HTTP origin, and every request to it costs a full connect
    timeout. [`ChassisState::available`] is the daemon's standing answer to
    "is the base there", decided by a bounded startup probe and re-checked on a
    timer, so a caller can skip its chassis steps outright rather than
    discovering the absence one timeout at a time.

        Attributes:
            availability_reason (str): Why [`ChassisState::available`] holds its current value: the transport
                error for an absent base, the configuration for one declared absent,
                and a short confirmation for a reachable one.
            available (bool): Whether the chassis controller answered the last availability check.

                `false` means every chassis command and query is refused immediately
                with `unavailable` and NOTHING is sent to the network; the rest of this
                structure is then the default reading, not a measurement. Consumers
                that orchestrate the robot should read this field and skip their
                chassis steps rather than issuing commands that cannot succeed.
            battery_percent (float): Battery percentage, or `-1.0` when unknown.
            bumper_pressed (bool): Whether a bumper is pressed.
            emergency_stop (bool): Whether the chassis emergency stop is active.
            last_probe_unix_ms (int): Milliseconds since the UNIX epoch at which the last availability check
                settled, or `0` when none has run (a backend that does not probe).
            navigating (bool): Whether navigation is currently active.
            status_json (str): Full vendor status JSON, preserved for frontend consumers.
            work_mode (WorkModeType0 | WorkModeType1 | WorkModeType2 | WorkModeType3 | WorkModeType4 | WorkModeType5): The
                decoded chassis work mode.
    """

    availability_reason: str
    available: bool
    battery_percent: float
    bumper_pressed: bool
    emergency_stop: bool
    last_probe_unix_ms: int
    navigating: bool
    status_json: str
    work_mode: (
        WorkModeType0
        | WorkModeType1
        | WorkModeType2
        | WorkModeType3
        | WorkModeType4
        | WorkModeType5
    )
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        availability_reason = self.availability_reason

        available = self.available

        battery_percent = self.battery_percent

        bumper_pressed = self.bumper_pressed

        emergency_stop = self.emergency_stop

        last_probe_unix_ms = self.last_probe_unix_ms

        navigating = self.navigating

        status_json = self.status_json

        work_mode: dict[str, Any] | str
        if (
            isinstance(self.work_mode, WorkModeType0)
            or isinstance(self.work_mode, WorkModeType1)
            or isinstance(self.work_mode, WorkModeType2)
            or isinstance(self.work_mode, WorkModeType3)
            or isinstance(self.work_mode, WorkModeType4)
        ):
            work_mode = self.work_mode.value
        else:
            work_mode = self.work_mode.to_dict()

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "availability_reason": availability_reason,
                "available": available,
                "battery_percent": battery_percent,
                "bumper_pressed": bumper_pressed,
                "emergency_stop": emergency_stop,
                "last_probe_unix_ms": last_probe_unix_ms,
                "navigating": navigating,
                "status_json": status_json,
                "work_mode": work_mode,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.work_mode_type_5 import WorkModeType5

        d = dict(src_dict)
        availability_reason = d.pop("availability_reason")

        available = d.pop("available")

        battery_percent = d.pop("battery_percent")

        bumper_pressed = d.pop("bumper_pressed")

        emergency_stop = d.pop("emergency_stop")

        last_probe_unix_ms = d.pop("last_probe_unix_ms")

        navigating = d.pop("navigating")

        status_json = d.pop("status_json")

        def _parse_work_mode(
            data: object,
        ) -> (
            WorkModeType0
            | WorkModeType1
            | WorkModeType2
            | WorkModeType3
            | WorkModeType4
            | WorkModeType5
        ):
            try:
                if not isinstance(data, str):
                    raise TypeError()
                componentsschemas_work_mode_type_0 = WorkModeType0(data)

                return componentsschemas_work_mode_type_0
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            try:
                if not isinstance(data, str):
                    raise TypeError()
                componentsschemas_work_mode_type_1 = WorkModeType1(data)

                return componentsschemas_work_mode_type_1
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            try:
                if not isinstance(data, str):
                    raise TypeError()
                componentsschemas_work_mode_type_2 = WorkModeType2(data)

                return componentsschemas_work_mode_type_2
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            try:
                if not isinstance(data, str):
                    raise TypeError()
                componentsschemas_work_mode_type_3 = WorkModeType3(data)

                return componentsschemas_work_mode_type_3
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            try:
                if not isinstance(data, str):
                    raise TypeError()
                componentsschemas_work_mode_type_4 = WorkModeType4(data)

                return componentsschemas_work_mode_type_4
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            if not isinstance(data, dict):
                raise TypeError()
            componentsschemas_work_mode_type_5 = WorkModeType5.from_dict(data)

            return componentsschemas_work_mode_type_5

        work_mode = _parse_work_mode(d.pop("work_mode"))

        chassis_state = cls(
            availability_reason=availability_reason,
            available=available,
            battery_percent=battery_percent,
            bumper_pressed=bumper_pressed,
            emergency_stop=emergency_stop,
            last_probe_unix_ms=last_probe_unix_ms,
            navigating=navigating,
            status_json=status_json,
            work_mode=work_mode,
        )

        chassis_state.additional_properties = d
        return chassis_state

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
