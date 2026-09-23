from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.arm_brake_engage_reason import ArmBrakeEngageReason
from ..models.arm_side import ArmSide
from ..types import UNSET, Unset

T = TypeVar("T", bound="ArmBrakeReport")


@_attrs_define
class ArmBrakeReport:
    """One arm's holding-brake state as this daemon commanded it.

    Attributes:
        released (bool): Whether this daemon has forced this arm's holding brakes OPEN and not
            closed them since. While `true`, nothing holds the arm but whoever is
            supporting it.

            The controller acknowledges brake writes but does not report the brake
            state back, so this is the daemon's command record, not a sensor.
        side (ArmSide): Selects one of the two physical arms.
        holder (None | str | Unset): The arm-lease holder that asked for the current release, when the
            request named one.
        last_engage_reason (ArmBrakeEngageReason | None | Unset):
        remaining_s (float | None | Unset): Seconds left before the daemon re-engages the brakes itself. `None`
            while engaged; `0` if the window ran out and the re-engage has not
            been confirmed yet (see the daemon log).
        window_s (float | None | Unset): The window the current release was granted, in seconds. `None` while
            engaged.
    """

    released: bool
    side: ArmSide
    holder: None | str | Unset = UNSET
    last_engage_reason: ArmBrakeEngageReason | None | Unset = UNSET
    remaining_s: float | None | Unset = UNSET
    window_s: float | None | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        released = self.released

        side = self.side.value

        holder: None | str | Unset
        if isinstance(self.holder, Unset):
            holder = UNSET
        else:
            holder = self.holder

        last_engage_reason: None | str | Unset
        if isinstance(self.last_engage_reason, Unset):
            last_engage_reason = UNSET
        elif isinstance(self.last_engage_reason, ArmBrakeEngageReason):
            last_engage_reason = self.last_engage_reason.value
        else:
            last_engage_reason = self.last_engage_reason

        remaining_s: float | None | Unset
        if isinstance(self.remaining_s, Unset):
            remaining_s = UNSET
        else:
            remaining_s = self.remaining_s

        window_s: float | None | Unset
        if isinstance(self.window_s, Unset):
            window_s = UNSET
        else:
            window_s = self.window_s

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "released": released,
                "side": side,
            }
        )
        if holder is not UNSET:
            field_dict["holder"] = holder
        if last_engage_reason is not UNSET:
            field_dict["last_engage_reason"] = last_engage_reason
        if remaining_s is not UNSET:
            field_dict["remaining_s"] = remaining_s
        if window_s is not UNSET:
            field_dict["window_s"] = window_s

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        released = d.pop("released")

        side = ArmSide(d.pop("side"))

        def _parse_holder(data: object) -> None | str | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(None | str | Unset, data)

        holder = _parse_holder(d.pop("holder", UNSET))

        def _parse_last_engage_reason(
            data: object,
        ) -> ArmBrakeEngageReason | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            try:
                if not isinstance(data, str):
                    raise TypeError()
                last_engage_reason_type_1 = ArmBrakeEngageReason(data)

                return last_engage_reason_type_1
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            return cast(ArmBrakeEngageReason | None | Unset, data)

        last_engage_reason = _parse_last_engage_reason(
            d.pop("last_engage_reason", UNSET)
        )

        def _parse_remaining_s(data: object) -> float | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(float | None | Unset, data)

        remaining_s = _parse_remaining_s(d.pop("remaining_s", UNSET))

        def _parse_window_s(data: object) -> float | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(float | None | Unset, data)

        window_s = _parse_window_s(d.pop("window_s", UNSET))

        arm_brake_report = cls(
            released=released,
            side=side,
            holder=holder,
            last_engage_reason=last_engage_reason,
            remaining_s=remaining_s,
            window_s=window_s,
        )

        arm_brake_report.additional_properties = d
        return arm_brake_report

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
