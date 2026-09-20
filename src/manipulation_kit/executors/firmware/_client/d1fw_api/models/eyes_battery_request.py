from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..types import UNSET, Unset

T = TypeVar("T", bound="EyesBatteryRequest")


@_attrs_define
class EyesBatteryRequest:
    """`POST /v1/eyes/battery_level`.

    Attributes:
        percent (float | None | Unset): Battery percentage `0..=100`, clamped into that range.  `null` clears
            the gauge.  Non-finite values may also be sent as the strings `nan`,
            `inf` or `-inf`, which JSON itself cannot represent.
    """

    percent: float | None | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        percent: float | None | Unset
        if isinstance(self.percent, Unset):
            percent = UNSET
        else:
            percent = self.percent

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update({})
        if percent is not UNSET:
            field_dict["percent"] = percent

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)

        def _parse_percent(data: object) -> float | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(float | None | Unset, data)

        percent = _parse_percent(d.pop("percent", UNSET))

        eyes_battery_request = cls(
            percent=percent,
        )

        eyes_battery_request.additional_properties = d
        return eyes_battery_request

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
