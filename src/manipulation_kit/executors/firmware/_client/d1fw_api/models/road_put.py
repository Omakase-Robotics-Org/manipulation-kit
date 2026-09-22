from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

if TYPE_CHECKING:
    from ..models.road_put_line_lists_item import RoadPutLineListsItem
    from ..models.road_put_point_list_item import RoadPutPointListItem


T = TypeVar("T", bound="RoadPut")


@_attrs_define
class RoadPut:
    """`PUT /v1/chassis/maps/{scene}/road`.

    Whole-resource replacement: the two lists are the scene's complete road
    network afterwards, not a delta. Both are forwarded to the mobile base as
    opaque JSON, because the base's own description of its `saveMapRoad` body
    declares them as bare lists with no element type, so this daemon does not
    invent a shape the base does not attest.

        Attributes:
            line_lists (list[RoadPutLineListsItem]): The edges joining those waypoints, in the base's own edge shape.
            point_list (list[RoadPutPointListItem]): The scene's waypoints, in the base's own point shape.
    """

    line_lists: list[RoadPutLineListsItem]
    point_list: list[RoadPutPointListItem]
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        line_lists = []
        for line_lists_item_data in self.line_lists:
            line_lists_item = line_lists_item_data.to_dict()
            line_lists.append(line_lists_item)

        point_list = []
        for point_list_item_data in self.point_list:
            point_list_item = point_list_item_data.to_dict()
            point_list.append(point_list_item)

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "lineLists": line_lists,
                "pointList": point_list,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.road_put_line_lists_item import (
            RoadPutLineListsItem,
        )
        from ..models.road_put_point_list_item import (
            RoadPutPointListItem,
        )

        d = dict(src_dict)
        line_lists = []
        _line_lists = d.pop("lineLists")
        for line_lists_item_data in _line_lists:
            line_lists_item = RoadPutLineListsItem.from_dict(line_lists_item_data)

            line_lists.append(line_lists_item)

        point_list = []
        _point_list = d.pop("pointList")
        for point_list_item_data in _point_list:
            point_list_item = RoadPutPointListItem.from_dict(point_list_item_data)

            point_list.append(point_list_item)

        road_put = cls(
            line_lists=line_lists,
            point_list=point_list,
        )

        road_put.additional_properties = d
        return road_put

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
