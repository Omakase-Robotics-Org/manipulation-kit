from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

T = TypeVar("T", bound="SliderSetZeroRequest")


@_attrs_define
class SliderSetZeroRequest:
    """`POST /v1/slider/set_zero`.

    Attributes:
        confirm (str): Must be exactly `SET_ZERO`. Anything else is a `400`, and the body is
            required: this verb throws away the lift's origin, and an empty POST
            is how a console button or a curl retry reaches a route by accident.
    """

    confirm: str
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        confirm = self.confirm

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "confirm": confirm,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        confirm = d.pop("confirm")

        slider_set_zero_request = cls(
            confirm=confirm,
        )

        slider_set_zero_request.additional_properties = d
        return slider_set_zero_request

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
