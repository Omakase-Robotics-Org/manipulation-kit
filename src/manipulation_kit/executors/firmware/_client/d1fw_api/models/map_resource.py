from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

if TYPE_CHECKING:
    from ..models.map_resource_waypoints_item import MapResourceWaypointsItem


T = TypeVar("T", bound="MapResource")


@_attrs_define
class MapResource:
    """One saved scene, with its waypoints.

    The same header fields as [`MapSummary`] plus the waypoint list itself.
    The waypoints are forwarded exactly as the mobile base's own interface
    produced them and are not reshaped: the shape belongs to the base, and a
    caller drawing a map needs every field the base kept, including ones this
    crate has no name for.

        Attributes:
            origin (list[float]): Map origin `[x, y, z]` in the scene's coordinate frame.
            resolution (float): Metres per pixel of the scene's map image.
            scene (str): The vendor scene name, which is this resource's identifier.
            waypoints (list[MapResourceWaypointsItem]): The scene's waypoints, forwarded unchanged from the base.
    """

    origin: list[float]
    resolution: float
    scene: str
    waypoints: list[MapResourceWaypointsItem]
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        origin = self.origin

        resolution = self.resolution

        scene = self.scene

        waypoints = []
        for waypoints_item_data in self.waypoints:
            waypoints_item = waypoints_item_data.to_dict()
            waypoints.append(waypoints_item)

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "origin": origin,
                "resolution": resolution,
                "scene": scene,
                "waypoints": waypoints,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.map_resource_waypoints_item import (
            MapResourceWaypointsItem,
        )

        d = dict(src_dict)
        origin = cast(list[float], d.pop("origin"))

        resolution = d.pop("resolution")

        scene = d.pop("scene")

        waypoints = []
        _waypoints = d.pop("waypoints")
        for waypoints_item_data in _waypoints:
            waypoints_item = MapResourceWaypointsItem.from_dict(waypoints_item_data)

            waypoints.append(waypoints_item)

        map_resource = cls(
            origin=origin,
            resolution=resolution,
            scene=scene,
            waypoints=waypoints,
        )

        map_resource.additional_properties = d
        return map_resource

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
