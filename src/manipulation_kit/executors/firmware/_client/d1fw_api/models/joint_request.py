from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

T = TypeVar("T", bound="JointRequest")


@_attrs_define
class JointRequest:
    """`POST /v1/arm/{side}/move_joint`.

    Attributes:
        joint (int): Zero-based joint index, `0..=6`.
        position (float): Target angle for that joint in degrees.
        wait (bool): Block until the arm reports arrival (up to 30 s).
    """

    joint: int
    position: float
    wait: bool
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        joint = self.joint

        position = self.position

        wait = self.wait

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "joint": joint,
                "position": position,
                "wait": wait,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        joint = d.pop("joint")

        position = d.pop("position")

        wait = d.pop("wait")

        joint_request = cls(
            joint=joint,
            position=position,
            wait=wait,
        )

        joint_request.additional_properties = d
        return joint_request

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
