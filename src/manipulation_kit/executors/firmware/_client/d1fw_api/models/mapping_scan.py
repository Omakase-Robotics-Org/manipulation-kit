from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

if TYPE_CHECKING:
    from ..models.mapping_stamp import MappingStamp


T = TypeVar("T", bound="MappingScan")


@_attrs_define
class MappingScan:
    """Frame-local laser scan. No map transform is inferred.

    Attributes:
        angle_increment (float): Angular separation in radians.
        angle_max (float): Received last-angle metadata in radians.
        angle_min (float): First ray angle in radians.
        frame_id (str): Received scan frame.
        range_max (float): Maximum valid range in metres.
        range_min (float): Minimum valid range in metres.
        ranges (list[float | None]): Metres; null means invalid/no return, never a zero-distance obstacle.
        stamp (MappingStamp): ROS source timestamp; distinct from receipt time and not a freshness guarantee.
    """

    angle_increment: float
    angle_max: float
    angle_min: float
    frame_id: str
    range_max: float
    range_min: float
    ranges: list[float | None]
    stamp: MappingStamp
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        angle_increment = self.angle_increment

        angle_max = self.angle_max

        angle_min = self.angle_min

        frame_id = self.frame_id

        range_max = self.range_max

        range_min = self.range_min

        ranges = []
        for ranges_item_data in self.ranges:
            ranges_item: float | None
            ranges_item = ranges_item_data
            ranges.append(ranges_item)

        stamp = self.stamp.to_dict()

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "angle_increment": angle_increment,
                "angle_max": angle_max,
                "angle_min": angle_min,
                "frame_id": frame_id,
                "range_max": range_max,
                "range_min": range_min,
                "ranges": ranges,
                "stamp": stamp,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.mapping_stamp import MappingStamp

        d = dict(src_dict)
        angle_increment = d.pop("angle_increment")

        angle_max = d.pop("angle_max")

        angle_min = d.pop("angle_min")

        frame_id = d.pop("frame_id")

        range_max = d.pop("range_max")

        range_min = d.pop("range_min")

        ranges = []
        _ranges = d.pop("ranges")
        for ranges_item_data in _ranges:

            def _parse_ranges_item(data: object) -> float | None:
                if data is None:
                    return data
                return cast(float | None, data)

            ranges_item = _parse_ranges_item(ranges_item_data)

            ranges.append(ranges_item)

        stamp = MappingStamp.from_dict(d.pop("stamp"))

        mapping_scan = cls(
            angle_increment=angle_increment,
            angle_max=angle_max,
            angle_min=angle_min,
            frame_id=frame_id,
            range_max=range_max,
            range_min=range_min,
            ranges=ranges,
            stamp=stamp,
        )

        mapping_scan.additional_properties = d
        return mapping_scan

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
