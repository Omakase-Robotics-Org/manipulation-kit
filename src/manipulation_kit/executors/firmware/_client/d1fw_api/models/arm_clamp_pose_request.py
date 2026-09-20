from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..types import UNSET, Unset

T = TypeVar("T", bound="ArmClampPoseRequest")


@_attrs_define
class ArmClampPoseRequest:
    """`POST /v1/arm/{side}/clamp_pose`: a read-only joint-limit clamp query.

    Attributes:
        joints_deg (list[float] | Unset): Joint angles in degrees; seven values.
    """

    joints_deg: list[float] | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        joints_deg: list[float] | Unset = UNSET
        if not isinstance(self.joints_deg, Unset):
            joints_deg = self.joints_deg

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update({})
        if joints_deg is not UNSET:
            field_dict["joints_deg"] = joints_deg

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        joints_deg = cast(list[float], d.pop("joints_deg", UNSET))

        arm_clamp_pose_request = cls(
            joints_deg=joints_deg,
        )

        arm_clamp_pose_request.additional_properties = d
        return arm_clamp_pose_request

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
