from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.charge_basis import ChargeBasis
from ..types import UNSET, Unset

if TYPE_CHECKING:
    from ..models.battery_cell import BatteryCell


T = TypeVar("T", bound="ChassisBattery")


@_attrs_define
class ChassisBattery:
    """`GET /v1/chassis/battery`.

    The single number this route has always returned, plus the reading it was
    taken from: which transport supplied it, and the per-pack detail when that
    transport reports any.  The two mobile-base firmware generations read the
    battery over different transports, so a consumer that must tell "no
    reading" from "a reading of zero" reads `source` and `percent` rather than
    `battery_percent` alone.

        Attributes:
            basis (ChargeBasis): Where the daemon's `charging` answer came from, or that it has no source.

                The two mobile-base firmware generations report charge over different
                transports and only one of them is right for a given base, so the evidence
                travels with the answer rather than being left for a consumer to guess.
            battery_percent (float): Battery percentage, or `-1` when the chassis did not report one.
            cells (list[BatteryCell]): The per-pack readings, empty on a source that has none.
            source (str): `http` or `sensor-stream`: where the charge is supposed to come from.
            charging (bool | None | Unset): Whether the base is charging: `true` and `false` only on positive
                evidence, `null` when the only evidence is a code the vendor
                documents nowhere or when there is none at all.

                Never `false` for an undocumented pack code. Read `basis` to see why
                a `null` is null.
            charging_status_raw (None | str | Unset): The vendor HTTP API's `chargingStatus` string exactly as it arrived,
                or `null` on a mobile-base firmware generation whose `chargingStatus`
                is a dead constant and is therefore not read at all.
            percent (float | None | Unset): The charge from that source, or `null` when it could not be read.
    """

    basis: ChargeBasis
    battery_percent: float
    cells: list[BatteryCell]
    source: str
    charging: bool | None | Unset = UNSET
    charging_status_raw: None | str | Unset = UNSET
    percent: float | None | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        basis = self.basis.value

        battery_percent = self.battery_percent

        cells = []
        for cells_item_data in self.cells:
            cells_item = cells_item_data.to_dict()
            cells.append(cells_item)

        source = self.source

        charging: bool | None | Unset
        if isinstance(self.charging, Unset):
            charging = UNSET
        else:
            charging = self.charging

        charging_status_raw: None | str | Unset
        if isinstance(self.charging_status_raw, Unset):
            charging_status_raw = UNSET
        else:
            charging_status_raw = self.charging_status_raw

        percent: float | None | Unset
        if isinstance(self.percent, Unset):
            percent = UNSET
        else:
            percent = self.percent

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "basis": basis,
                "battery_percent": battery_percent,
                "cells": cells,
                "source": source,
            }
        )
        if charging is not UNSET:
            field_dict["charging"] = charging
        if charging_status_raw is not UNSET:
            field_dict["charging_status_raw"] = charging_status_raw
        if percent is not UNSET:
            field_dict["percent"] = percent

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.battery_cell import BatteryCell

        d = dict(src_dict)
        basis = ChargeBasis(d.pop("basis"))

        battery_percent = d.pop("battery_percent")

        cells = []
        _cells = d.pop("cells")
        for cells_item_data in _cells:
            cells_item = BatteryCell.from_dict(cells_item_data)

            cells.append(cells_item)

        source = d.pop("source")

        def _parse_charging(data: object) -> bool | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(bool | None | Unset, data)

        charging = _parse_charging(d.pop("charging", UNSET))

        def _parse_charging_status_raw(data: object) -> None | str | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(None | str | Unset, data)

        charging_status_raw = _parse_charging_status_raw(
            d.pop("charging_status_raw", UNSET)
        )

        def _parse_percent(data: object) -> float | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(float | None | Unset, data)

        percent = _parse_percent(d.pop("percent", UNSET))

        chassis_battery = cls(
            basis=basis,
            battery_percent=battery_percent,
            cells=cells,
            source=source,
            charging=charging,
            charging_status_raw=charging_status_raw,
            percent=percent,
        )

        chassis_battery.additional_properties = d
        return chassis_battery

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
