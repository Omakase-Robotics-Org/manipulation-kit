from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..types import UNSET, Unset

T = TypeVar("T", bound="ArmMoveJointBody")


@_attrs_define
class ArmMoveJointBody:
    """
    Attributes:
        joint (int): Zero-based joint index, `0..=6`.
        position (float): Target angle for that joint in degrees.
        wait (bool): Block until the arm reports arrival (up to 30 s).
        holder (str | Unset): The arm lease holder issuing this command. Required only while somebody holds the lease:
            with no lease held the field is ignored, and with one held a request whose `holder` does not match is refused
            with 409.
    """

    joint: int
    position: float
    wait: bool
    holder: str | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        joint = self.joint

        position = self.position

        wait = self.wait

        holder = self.holder

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "joint": joint,
                "position": position,
                "wait": wait,
            }
        )
        if holder is not UNSET:
            field_dict["holder"] = holder

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        joint = d.pop("joint")

        position = d.pop("position")

        wait = d.pop("wait")

        holder = d.pop("holder", UNSET)

        arm_move_joint_body = cls(
            joint=joint,
            position=position,
            wait=wait,
            holder=holder,
        )

        arm_move_joint_body.additional_properties = d
        return arm_move_joint_body

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
