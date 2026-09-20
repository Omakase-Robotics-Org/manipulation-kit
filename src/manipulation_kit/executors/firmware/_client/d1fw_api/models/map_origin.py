from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

T = TypeVar("T", bound="MapOrigin")


@_attrs_define
class MapOrigin:
    """A saved scene's origin and resolution, from vendor `getOrigin`.

    Attributes:
        origin (list[float]): Map origin `[x, y, z]` in the scene's coordinate frame, as reported by
            the vendor.
        resolution (float): Metres per pixel.
    """

    origin: list[float]
    resolution: float
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        origin = self.origin

        resolution = self.resolution

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "origin": origin,
                "resolution": resolution,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        origin = cast(list[float], d.pop("origin"))

        resolution = d.pop("resolution")

        map_origin = cls(
            origin=origin,
            resolution=resolution,
        )

        map_origin.additional_properties = d
        return map_origin

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
