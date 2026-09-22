from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

T = TypeVar("T", bound="SliderTravelRange")


@_attrs_define
class SliderTravelRange:
    """One closed interval of lift travel in metres.

    Attributes:
        lower_m (float): The lowest height this layer admits.
        upper_m (float): The highest height this layer admits.
    """

    lower_m: float
    upper_m: float
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        lower_m = self.lower_m

        upper_m = self.upper_m

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "lower_m": lower_m,
                "upper_m": upper_m,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        lower_m = d.pop("lower_m")

        upper_m = d.pop("upper_m")

        slider_travel_range = cls(
            lower_m=lower_m,
            upper_m=upper_m,
        )

        slider_travel_range.additional_properties = d
        return slider_travel_range

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
