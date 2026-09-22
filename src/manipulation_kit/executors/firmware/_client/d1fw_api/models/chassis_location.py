from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..types import UNSET, Unset

T = TypeVar("T", bound="ChassisLocation")


@_attrs_define
class ChassisLocation:
    """Measured two-dimensional vendor pose in the current map's meter frame.

    Attributes:
        x_m (float): Measured map x coordinate in meters.
        y_m (float): Measured map y coordinate in meters.
        scene (None | str | Unset): Vendor map identity; absent when the vendor supplied none.
    """

    x_m: float
    y_m: float
    scene: None | str | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        x_m = self.x_m

        y_m = self.y_m

        scene: None | str | Unset
        if isinstance(self.scene, Unset):
            scene = UNSET
        else:
            scene = self.scene

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "x_m": x_m,
                "y_m": y_m,
            }
        )
        if scene is not UNSET:
            field_dict["scene"] = scene

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        x_m = d.pop("x_m")

        y_m = d.pop("y_m")

        def _parse_scene(data: object) -> None | str | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(None | str | Unset, data)

        scene = _parse_scene(d.pop("scene", UNSET))

        chassis_location = cls(
            x_m=x_m,
            y_m=y_m,
            scene=scene,
        )

        chassis_location.additional_properties = d
        return chassis_location

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
