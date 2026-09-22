from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

T = TypeVar("T", bound="MapSummary")


@_attrs_define
class MapSummary:
    """One saved scene as the map collection lists it.

    The typed summary of a scene: enough to draw a list, place the scene's
    image in metres and say how many waypoints it holds, without fetching the
    waypoints themselves. Composed from the vendor's scene list, its
    per-scene origin and its per-scene waypoint list.

        Attributes:
            origin (list[float]): Map origin `[x, y, z]` in the scene's coordinate frame.
            point_count (int): How many waypoints the scene's road network holds.
            resolution (float): Metres per pixel of the scene's map image.
            scene (str): The vendor scene name, which is this resource's identifier.
    """

    origin: list[float]
    point_count: int
    resolution: float
    scene: str
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        origin = self.origin

        point_count = self.point_count

        resolution = self.resolution

        scene = self.scene

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "origin": origin,
                "point_count": point_count,
                "resolution": resolution,
                "scene": scene,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        origin = cast(list[float], d.pop("origin"))

        point_count = d.pop("point_count")

        resolution = d.pop("resolution")

        scene = d.pop("scene")

        map_summary = cls(
            origin=origin,
            point_count=point_count,
            resolution=resolution,
            scene=scene,
        )

        map_summary.additional_properties = d
        return map_summary

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
