from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

if TYPE_CHECKING:
    from ..models.splice_area_put_points_item import SpliceAreaPutPointsItem


T = TypeVar("T", bound="SpliceAreaPut")


@_attrs_define
class SpliceAreaPut:
    """One typed special zone inside [`KeepOutsPut`]: a polygon plus the
    vendor's zone code (e.g. `"3"` ramp, `"4"` elevator, `"5"` slow-down,
    `"6"` bright-light, `"7"` obstacle-stop, `"8"` narrow).

        Attributes:
            points (list[SpliceAreaPutPointsItem]): The zone's polygon, in the scene's metre frame.
            type_ (str): The vendor zone code.
    """

    points: list[SpliceAreaPutPointsItem]
    type_: str
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        points = []
        for points_item_data in self.points:
            points_item = points_item_data.to_dict()
            points.append(points_item)

        type_ = self.type_

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "points": points,
                "type": type_,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.splice_area_put_points_item import (
            SpliceAreaPutPointsItem,
        )

        d = dict(src_dict)
        points = []
        _points = d.pop("points")
        for points_item_data in _points:
            points_item = SpliceAreaPutPointsItem.from_dict(points_item_data)

            points.append(points_item)

        type_ = d.pop("type")

        splice_area_put = cls(
            points=points,
            type_=type_,
        )

        splice_area_put.additional_properties = d
        return splice_area_put

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
