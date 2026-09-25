from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..types import UNSET, Unset

T = TypeVar("T", bound="BatteryCell")


@_attrs_define
class BatteryCell:
    """One battery pack of the mobile base, as the vendor's sensor push service
    reports it.

    Only the sensor-stream source produces these; the HTTP source has one
    aggregate percentage and no per-pack detail, so a [`ChassisBattery`] from
    it carries an empty `cells`.

        Attributes:
            age_ms (int): How long ago this pack's reading was taken, in milliseconds.

                `0` means the pack reported during the read that produced this state.
                A larger value means it did not, and the daemon is still reporting the
                last value that pack sent.

                The base pushes one pack per frame, so a read of the push service can
                legitimately come back with only one of the two packs in it -- the
                service serves one client at a time and evicts the previous one, and a
                read that is cut short that way is not evidence that a pack is gone.
                Dropping the missing pack from the reading made the published charge
                oscillate: on the D1 mobile base measured on 2026-09-19, `cells`
                alternated between two packs and one every few seconds and the pack
                aggregate followed it. So the last reading of each pack is retained
                for a bounded time and published with its age, which is the fact a
                consumer needs in order to decide whether to trust it.
            charge_status (int): The pack's raw `charge_status` field exactly as the vendor's sensor
                push service sent it. `0` discharging and `1` charging are the only two
                the vendor documents.
            current (float): Pack current, in the vendor's own raw unit.
            max_temperature (float): This pack's highest cell temperature, in the vendor's own raw unit
                (degrees Celsius in every reading measured so far).
            remaining_capacity (float): Remaining capacity, in the vendor's own raw unit.
            serial (int): The vendor's `serial_number`: 0 is pack 1, 1 is pack 2.
            soc_percent (float): This pack's state of charge, as a percentage.
            voltage (float): Pack voltage in volts.
            charging (bool | None | Unset): Whether this pack reports current flowing in: `true` for the vendor's
                documented `charge_status` 1, `false` for the documented 0, and `null`
                for any other value the pack reports.

                Never `false` for an undocumented code. The D1 mobile base measured on
                2026-09-09 reports `charge_status = 2`, which the vendor documents
                nowhere; answering `false` there would tell an operator the pack is not
                charging when nothing measured says so.
    """

    age_ms: int
    charge_status: int
    current: float
    max_temperature: float
    remaining_capacity: float
    serial: int
    soc_percent: float
    voltage: float
    charging: bool | None | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        age_ms = self.age_ms

        charge_status = self.charge_status

        current = self.current

        max_temperature = self.max_temperature

        remaining_capacity = self.remaining_capacity

        serial = self.serial

        soc_percent = self.soc_percent

        voltage = self.voltage

        charging: bool | None | Unset
        if isinstance(self.charging, Unset):
            charging = UNSET
        else:
            charging = self.charging

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "age_ms": age_ms,
                "charge_status": charge_status,
                "current": current,
                "max_temperature": max_temperature,
                "remaining_capacity": remaining_capacity,
                "serial": serial,
                "soc_percent": soc_percent,
                "voltage": voltage,
            }
        )
        if charging is not UNSET:
            field_dict["charging"] = charging

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        age_ms = d.pop("age_ms")

        charge_status = d.pop("charge_status")

        current = d.pop("current")

        max_temperature = d.pop("max_temperature")

        remaining_capacity = d.pop("remaining_capacity")

        serial = d.pop("serial")

        soc_percent = d.pop("soc_percent")

        voltage = d.pop("voltage")

        def _parse_charging(data: object) -> bool | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(bool | None | Unset, data)

        charging = _parse_charging(d.pop("charging", UNSET))

        battery_cell = cls(
            age_ms=age_ms,
            charge_status=charge_status,
            current=current,
            max_temperature=max_temperature,
            remaining_capacity=remaining_capacity,
            serial=serial,
            soc_percent=soc_percent,
            voltage=voltage,
            charging=charging,
        )

        battery_cell.additional_properties = d
        return battery_cell

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
