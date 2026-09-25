from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

if TYPE_CHECKING:
    from ..models.mapping_stamp import MappingStamp


T = TypeVar("T", bound="MappingGrid")


@_attrs_define
class MappingGrid:
    """Typed ROS occupancy grid, with x varying fastest and y increasing by row.

    Attributes:
        cells (list[int]): -1 unknown, 0 free through 100 occupied; exactly width*height cells.
        frame_id (str): Received coordinate frame; never assumed map.
        height (int): Number of rows.
        origin_orientation (list[float]): Origin quaternion x/y/z/w, validated unit length.
        origin_position (list[float]): Origin position x/y/z in received frame.
        resolution (float): Grid cell size in metres.
        stamp (MappingStamp): ROS source timestamp; distinct from receipt time and not a freshness guarantee.
        width (int): Cells per row.
    """

    cells: list[int]
    frame_id: str
    height: int
    origin_orientation: list[float]
    origin_position: list[float]
    resolution: float
    stamp: MappingStamp
    width: int
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        cells = self.cells

        frame_id = self.frame_id

        height = self.height

        origin_orientation = self.origin_orientation

        origin_position = self.origin_position

        resolution = self.resolution

        stamp = self.stamp.to_dict()

        width = self.width

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "cells": cells,
                "frame_id": frame_id,
                "height": height,
                "origin_orientation": origin_orientation,
                "origin_position": origin_position,
                "resolution": resolution,
                "stamp": stamp,
                "width": width,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.mapping_stamp import MappingStamp

        d = dict(src_dict)
        cells = cast(list[int], d.pop("cells"))

        frame_id = d.pop("frame_id")

        height = d.pop("height")

        origin_orientation = cast(list[float], d.pop("origin_orientation"))

        origin_position = cast(list[float], d.pop("origin_position"))

        resolution = d.pop("resolution")

        stamp = MappingStamp.from_dict(d.pop("stamp"))

        width = d.pop("width")

        mapping_grid = cls(
            cells=cells,
            frame_id=frame_id,
            height=height,
            origin_orientation=origin_orientation,
            origin_position=origin_position,
            resolution=resolution,
            stamp=stamp,
            width=width,
        )

        mapping_grid.additional_properties = d
        return mapping_grid

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
