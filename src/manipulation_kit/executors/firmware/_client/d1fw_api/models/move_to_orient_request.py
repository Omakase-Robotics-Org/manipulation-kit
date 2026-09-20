from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.chassis_orient import ChassisOrient

T = TypeVar("T", bound="MoveToOrientRequest")


@_attrs_define
class MoveToOrientRequest:
    """`POST /v1/chassis/move_to_orient`.

    Attributes:
        orient (ChassisOrient): A remote-control teleop direction, matching the vendor `sendMoveToOrient`
            endpoint's four accepted values.
    """

    orient: ChassisOrient
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        orient = self.orient.value

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "orient": orient,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        orient = ChassisOrient(d.pop("orient"))

        move_to_orient_request = cls(
            orient=orient,
        )

        move_to_orient_request.additional_properties = d
        return move_to_orient_request

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
