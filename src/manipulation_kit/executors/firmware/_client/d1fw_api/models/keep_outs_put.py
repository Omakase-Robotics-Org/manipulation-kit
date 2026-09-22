from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

if TYPE_CHECKING:
    from ..models.keep_outs_put_forbidden_areas_item_item import (
        KeepOutsPutForbiddenAreasItemItem,
    )
    from ..models.splice_area_put import SpliceAreaPut


T = TypeVar("T", bound="KeepOutsPut")


@_attrs_define
class KeepOutsPut:
    """`PUT /v1/chassis/maps/{scene}/keep_outs`.

    Whole-sub-resource replacement, matching [`RoadPut`]'s semantics: the two
    fields are the scene's complete keep-out and typed-zone geometry
    afterwards, not a delta, so writing both empty clears the scene. Each
    point in `forbidden_areas` and in a [`SpliceAreaPut`]'s `points` accepts
    either the base's own `[x, y]` array spelling or the `{"x": x, "y": y}`
    object spelling the keep-out `GET` serves inside each
    `forbiddenAreasTemp` string (`d1fw_core::Point`), so a caller that echoes
    a point it just read back into a write is accepted too.

        Attributes:
            forbidden_areas (list[list[KeepOutsPutForbiddenAreasItemItem]]): The keep-out geometry: one entry per area, each
                a list of `[x, y]`
                points (exactly two is a virtual wall segment, three or more a
                forbidden polygon).
            splice_areas (list[SpliceAreaPut]): The scene's typed special zones.
    """

    forbidden_areas: list[list[KeepOutsPutForbiddenAreasItemItem]]
    splice_areas: list[SpliceAreaPut]
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        forbidden_areas = []
        for forbidden_areas_item_data in self.forbidden_areas:
            forbidden_areas_item = []
            for forbidden_areas_item_item_data in forbidden_areas_item_data:
                forbidden_areas_item_item = forbidden_areas_item_item_data.to_dict()
                forbidden_areas_item.append(forbidden_areas_item_item)

            forbidden_areas.append(forbidden_areas_item)

        splice_areas = []
        for splice_areas_item_data in self.splice_areas:
            splice_areas_item = splice_areas_item_data.to_dict()
            splice_areas.append(splice_areas_item)

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "forbidden_areas": forbidden_areas,
                "splice_areas": splice_areas,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.keep_outs_put_forbidden_areas_item_item import (
            KeepOutsPutForbiddenAreasItemItem,
        )
        from ..models.splice_area_put import SpliceAreaPut

        d = dict(src_dict)
        forbidden_areas = []
        _forbidden_areas = d.pop("forbidden_areas")
        for forbidden_areas_item_data in _forbidden_areas:
            forbidden_areas_item = []
            _forbidden_areas_item = forbidden_areas_item_data
            for forbidden_areas_item_item_data in _forbidden_areas_item:
                forbidden_areas_item_item = KeepOutsPutForbiddenAreasItemItem.from_dict(
                    forbidden_areas_item_item_data
                )

                forbidden_areas_item.append(forbidden_areas_item_item)

            forbidden_areas.append(forbidden_areas_item)

        splice_areas = []
        _splice_areas = d.pop("splice_areas")
        for splice_areas_item_data in _splice_areas:
            splice_areas_item = SpliceAreaPut.from_dict(splice_areas_item_data)

            splice_areas.append(splice_areas_item)

        keep_outs_put = cls(
            forbidden_areas=forbidden_areas,
            splice_areas=splice_areas,
        )

        keep_outs_put.additional_properties = d
        return keep_outs_put

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
