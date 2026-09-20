from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..types import UNSET, Unset

T = TypeVar("T", bound="ArmCheckPoseRequest")


@_attrs_define
class ArmCheckPoseRequest:
    """`POST /v1/arm/check_pose`: a read-only guard query over both arms.

    Attributes:
        joints_a_deg (list[float] | Unset): Arm A joint angles in degrees; seven values, or empty to check arm B
            alone.
        joints_b_deg (list[float] | Unset): Arm B joint angles in degrees; seven values, or empty to check arm A
            alone.
    """

    joints_a_deg: list[float] | Unset = UNSET
    joints_b_deg: list[float] | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        joints_a_deg: list[float] | Unset = UNSET
        if not isinstance(self.joints_a_deg, Unset):
            joints_a_deg = self.joints_a_deg

        joints_b_deg: list[float] | Unset = UNSET
        if not isinstance(self.joints_b_deg, Unset):
            joints_b_deg = self.joints_b_deg

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update({})
        if joints_a_deg is not UNSET:
            field_dict["joints_a_deg"] = joints_a_deg
        if joints_b_deg is not UNSET:
            field_dict["joints_b_deg"] = joints_b_deg

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        joints_a_deg = cast(list[float], d.pop("joints_a_deg", UNSET))

        joints_b_deg = cast(list[float], d.pop("joints_b_deg", UNSET))

        arm_check_pose_request = cls(
            joints_a_deg=joints_a_deg,
            joints_b_deg=joints_b_deg,
        )

        arm_check_pose_request.additional_properties = d
        return arm_check_pose_request

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
