from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

T = TypeVar("T", bound="MappingStamp")


@_attrs_define
class MappingStamp:
    """ROS source timestamp; distinct from receipt time and not a freshness guarantee.

    Attributes:
        nsecs (int): Source nanoseconds, less than one billion.
        secs (int): Source seconds.
    """

    nsecs: int
    secs: int
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        nsecs = self.nsecs

        secs = self.secs

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "nsecs": nsecs,
                "secs": secs,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        nsecs = d.pop("nsecs")

        secs = d.pop("secs")

        mapping_stamp = cls(
            nsecs=nsecs,
            secs=secs,
        )

        mapping_stamp.additional_properties = d
        return mapping_stamp

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
