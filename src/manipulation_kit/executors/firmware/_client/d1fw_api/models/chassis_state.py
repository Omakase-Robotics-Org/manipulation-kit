from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.work_mode_type_0 import WorkModeType0
from ..models.work_mode_type_1 import WorkModeType1
from ..models.work_mode_type_2 import WorkModeType2
from ..models.work_mode_type_3 import WorkModeType3
from ..models.work_mode_type_4 import WorkModeType4
from ..types import UNSET, Unset

if TYPE_CHECKING:
    from ..models.chassis_battery_reading import ChassisBatteryReading
    from ..models.chassis_charge_state import ChassisChargeState
    from ..models.chassis_control_state import ChassisControlState
    from ..models.chassis_location import ChassisLocation
    from ..models.chassis_speed_cap import ChassisSpeedCap
    from ..models.chassis_vendor_info import ChassisVendorInfo
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
            battery (ChassisBatteryReading): The mobile base's battery reading, and which source it came from.

                The two mobile-base firmware generations report the battery over different
                transports, and only one of them is right for a given base: the older
                generation's HTTP `powerPercentage` field, or the newer generation's TCP
                sensor push service. This value says which source was used, so a consumer
                can tell "no reading" from "a reading of zero" and can see when a base's
                charge is coming from somewhere other than where it expects.

                [`ChassisState::battery_percent`] is filled from the SELECTED source and
                stays the single number a consumer that wants only a number reads.
            battery_percent (float): Battery percentage, or `-1.0` when unknown.
            bumper_pressed (bool): Whether a bumper is pressed.
            charge (ChassisChargeState): Everything the daemon knows about the mobile base's charging, in one
                object: where the base is in its docking routine, whether automatic
                charging is the mode it is in, what its battery packs say about current
                flow, and which waypoint pressing "charge" would send it to.

                The four are separate fields because they are four different measurements
                that routinely disagree, and folding them into one flag is what makes an
                operator guess. The vendor's own sequence, from its example program
                `7.add_charge.py`, is: start automatic charging, and `workMode` goes 1 to 2
                while `dockStatus` goes 0 to 1; docking completes and `dockStatus` becomes
                2; the base leaves and `dockStatus` goes 3 then 0 while `workMode` returns
                to 1. Through all of that, whether current is actually flowing is reported
                only by the battery pack.
            control (ChassisControlState): Everything the daemon knows about WHICH CONTROLLER may drive the mobile
                base: the mode it is in, what that mode permits, and which control verbs
                are legal from it.

                The mobile base has exactly one control mode at a time and every motion
                path is gated on it, but the vendor publishes that only as a bare
                `workMode` digit and enforces it by IGNORING commands: outside its
                remote-control mode the base answers a jog with HTTP 200 and does not
                move. This object is the daemon's answer to "what state is the base in,
                what can I do from here, and why not" in one read, so that an operator
                interface never has to infer it from a digit or from a command that
                silently did nothing.

                The five vendor modes are the vendor's own, taken from its web
                application's own translation table: `0` navigation closed, `1`
                navigation, `2` automatic charging, `3` mapping, `4` remote control.
            emergency_stop (bool): Whether the chassis emergency stop is active.
            last_probe_unix_ms (int): Milliseconds since the UNIX epoch at which the last availability check
                settled, or `0` when none has run (a backend that does not probe).
            navigating (bool): Whether navigation is currently active.
            speed_cap (ChassisSpeedCap): The effective linear speed cap, and the two caps it was chosen between.

                The mobile base saturates a velocity command silently: its navigation
                stack carries its own `max_linear_vel`, and a `sendToMove` above that
                value is accepted, answered with success, and then driven at the base's
                own cap instead. A single number in an operator interface is therefore not
                enough -- "I asked for 0.5 m/s and the base said yes" is true and useless.
                This structure reports the number a command is actually clamped to, both
                inputs to that choice, and which one bound.
            status_json (str): Full vendor status JSON, preserved for frontend consumers.
            vendor (ChassisVendorInfo): The mobile base's vendor-reported identity and firmware/software build
                strings, as returned by the chassis controller's `getRobotInfo` endpoint.

                These fields are informational only (nothing in this core acts on them);
                they exist so that a vendor identity or firmware change on the chassis PC
                — otherwise silent, since the daemon only ever polls state — is observable
                through the same state and telemetry surfaces as any other chassis
                reading.
            work_mode (WorkModeType0 | WorkModeType1 | WorkModeType2 | WorkModeType3 | WorkModeType4 | WorkModeType5): The
                decoded chassis work mode.
            location (ChassisLocation | None | Unset):
    """

    availability_reason: str
    available: bool
    battery: ChassisBatteryReading
    battery_percent: float
    bumper_pressed: bool
    charge: ChassisChargeState
    control: ChassisControlState
    emergency_stop: bool
    last_probe_unix_ms: int
    navigating: bool
    speed_cap: ChassisSpeedCap
    status_json: str
    vendor: ChassisVendorInfo
    work_mode: (
        WorkModeType0
        | WorkModeType1
        | WorkModeType2
        | WorkModeType3
        | WorkModeType4
        | WorkModeType5
    )
    location: ChassisLocation | None | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        from ..models.chassis_location import ChassisLocation

        availability_reason = self.availability_reason

        available = self.available

        battery = self.battery.to_dict()

        battery_percent = self.battery_percent

        bumper_pressed = self.bumper_pressed

        charge = self.charge.to_dict()

        control = self.control.to_dict()

        emergency_stop = self.emergency_stop

        last_probe_unix_ms = self.last_probe_unix_ms

        navigating = self.navigating

        speed_cap = self.speed_cap.to_dict()

        status_json = self.status_json

        vendor = self.vendor.to_dict()

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

        location: dict[str, Any] | None | Unset
        if isinstance(self.location, Unset):
            location = UNSET
        elif isinstance(self.location, ChassisLocation):
            location = self.location.to_dict()
        else:
            location = self.location

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "availability_reason": availability_reason,
                "available": available,
                "battery": battery,
                "battery_percent": battery_percent,
                "bumper_pressed": bumper_pressed,
                "charge": charge,
                "control": control,
                "emergency_stop": emergency_stop,
                "last_probe_unix_ms": last_probe_unix_ms,
                "navigating": navigating,
                "speed_cap": speed_cap,
                "status_json": status_json,
                "vendor": vendor,
                "work_mode": work_mode,
            }
        )
        if location is not UNSET:
            field_dict["location"] = location

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.chassis_battery_reading import (
            ChassisBatteryReading,
        )
        from ..models.chassis_charge_state import ChassisChargeState
        from ..models.chassis_control_state import ChassisControlState
        from ..models.chassis_location import ChassisLocation
        from ..models.chassis_speed_cap import ChassisSpeedCap
        from ..models.chassis_vendor_info import ChassisVendorInfo
        from ..models.work_mode_type_5 import WorkModeType5

        d = dict(src_dict)
        availability_reason = d.pop("availability_reason")

        available = d.pop("available")

        battery = ChassisBatteryReading.from_dict(d.pop("battery"))

        battery_percent = d.pop("battery_percent")

        bumper_pressed = d.pop("bumper_pressed")

        charge = ChassisChargeState.from_dict(d.pop("charge"))

        control = ChassisControlState.from_dict(d.pop("control"))

        emergency_stop = d.pop("emergency_stop")

        last_probe_unix_ms = d.pop("last_probe_unix_ms")

        navigating = d.pop("navigating")

        speed_cap = ChassisSpeedCap.from_dict(d.pop("speed_cap"))

        status_json = d.pop("status_json")

        vendor = ChassisVendorInfo.from_dict(d.pop("vendor"))

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

        def _parse_location(data: object) -> ChassisLocation | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            try:
                if not isinstance(data, dict):
                    raise TypeError()
                location_type_1 = ChassisLocation.from_dict(data)

                return location_type_1
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            return cast(ChassisLocation | None | Unset, data)

        location = _parse_location(d.pop("location", UNSET))

        chassis_state = cls(
            availability_reason=availability_reason,
            available=available,
            battery=battery,
            battery_percent=battery_percent,
            bumper_pressed=bumper_pressed,
            charge=charge,
            control=control,
            emergency_stop=emergency_stop,
            last_probe_unix_ms=last_probe_unix_ms,
            navigating=navigating,
            speed_cap=speed_cap,
            status_json=status_json,
            vendor=vendor,
            work_mode=work_mode,
            location=location,
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
