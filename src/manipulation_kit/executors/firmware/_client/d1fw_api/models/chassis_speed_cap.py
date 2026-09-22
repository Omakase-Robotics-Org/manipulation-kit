from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.chassis_speed_cap_source import ChassisSpeedCapSource

T = TypeVar("T", bound="ChassisSpeedCap")


@_attrs_define
class ChassisSpeedCap:
    """The effective linear speed cap, and the two caps it was chosen between.

    The mobile base saturates a velocity command silently: its navigation
    stack carries its own `max_linear_vel`, and a `sendToMove` above that
    value is accepted, answered with success, and then driven at the base's
    own cap instead. A single number in an operator interface is therefore not
    enough -- "I asked for 0.5 m/s and the base said yes" is true and useless.
    This structure reports the number a command is actually clamped to, both
    inputs to that choice, and which one bound.

        Attributes:
            daemon_linear_m_s (float): This daemon's pinned safety limit, independent of any base.
            effective_linear_m_s (float): The cap velocity commands are clamped to: the lower of the two below,
                or the daemon's own when the base's could not be read.
            source (ChassisSpeedCapSource): Which of the two linear speed caps turned out to be the binding one.
            vendor_linear_m_s (float | None): The base's own declared maximum, or null when it could not be read.

                Read from the `max` field of the vendor's saved speed settings. It is
                a per-generation and per-unit value -- the deployed navigation stack of
                one base declares 0.3 where another declares 1.0 -- so it is read from
                the base rather than pinned here.
            vendor_read_error (None | str): Why the base's declared maximum is absent, or null when it was read.
    """

    daemon_linear_m_s: float
    effective_linear_m_s: float
    source: ChassisSpeedCapSource
    vendor_linear_m_s: float | None
    vendor_read_error: None | str
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        daemon_linear_m_s = self.daemon_linear_m_s

        effective_linear_m_s = self.effective_linear_m_s

        source = self.source.value

        vendor_linear_m_s: float | None
        vendor_linear_m_s = self.vendor_linear_m_s

        vendor_read_error: None | str
        vendor_read_error = self.vendor_read_error

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "daemon_linear_m_s": daemon_linear_m_s,
                "effective_linear_m_s": effective_linear_m_s,
                "source": source,
                "vendor_linear_m_s": vendor_linear_m_s,
                "vendor_read_error": vendor_read_error,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        daemon_linear_m_s = d.pop("daemon_linear_m_s")

        effective_linear_m_s = d.pop("effective_linear_m_s")

        source = ChassisSpeedCapSource(d.pop("source"))

        def _parse_vendor_linear_m_s(data: object) -> float | None:
            if data is None:
                return data
            return cast(float | None, data)

        vendor_linear_m_s = _parse_vendor_linear_m_s(d.pop("vendor_linear_m_s"))

        def _parse_vendor_read_error(data: object) -> None | str:
            if data is None:
                return data
            return cast(None | str, data)

        vendor_read_error = _parse_vendor_read_error(d.pop("vendor_read_error"))

        chassis_speed_cap = cls(
            daemon_linear_m_s=daemon_linear_m_s,
            effective_linear_m_s=effective_linear_m_s,
            source=source,
            vendor_linear_m_s=vendor_linear_m_s,
            vendor_read_error=vendor_read_error,
        )

        chassis_speed_cap.additional_properties = d
        return chassis_speed_cap

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
