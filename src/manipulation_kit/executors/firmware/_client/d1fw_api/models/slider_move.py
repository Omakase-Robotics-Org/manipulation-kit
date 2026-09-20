from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

T = TypeVar("T", bound="SliderMove")


@_attrs_define
class SliderMove:
    """A requested slider height.

    Attributes:
        height_m (float): Absolute target height in metres.
        wait (bool): Wait for the backend to report completion when true.
    """

    height_m: float
    wait: bool
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        height_m = self.height_m

        wait = self.wait

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "height_m": height_m,
                "wait": wait,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        height_m = d.pop("height_m")

        wait = d.pop("wait")

        slider_move = cls(
            height_m=height_m,
            wait=wait,
        )

        slider_move.additional_properties = d
        return slider_move

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
