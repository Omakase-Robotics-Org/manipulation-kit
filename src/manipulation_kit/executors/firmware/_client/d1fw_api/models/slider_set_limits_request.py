from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

T = TypeVar("T", bound="SliderSetLimitsRequest")


@_attrs_define
class SliderSetLimitsRequest:
    """`POST /v1/slider/set_limits`.

    Attributes:
        confirm (str): Must be exactly `SET_LIMITS`. Anything else is a `400`.
        lower_m (float): The lowest height this unit may be commanded to, in metres. Not below
            `0`.
        upper_m (float): The highest height this unit may be commanded to, in metres. Not above
            the model ceiling `[slider] travel_max_m`: a unit calibration may only
            narrow.
    """

    confirm: str
    lower_m: float
    upper_m: float
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        confirm = self.confirm

        lower_m = self.lower_m

        upper_m = self.upper_m

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "confirm": confirm,
                "lower_m": lower_m,
                "upper_m": upper_m,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        confirm = d.pop("confirm")

        lower_m = d.pop("lower_m")

        upper_m = d.pop("upper_m")

        slider_set_limits_request = cls(
            confirm=confirm,
            lower_m=lower_m,
            upper_m=upper_m,
        )

        slider_set_limits_request.additional_properties = d
        return slider_set_limits_request

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
