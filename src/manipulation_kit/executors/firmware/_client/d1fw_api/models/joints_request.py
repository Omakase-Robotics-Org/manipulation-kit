from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

T = TypeVar("T", bound="JointsRequest")


@_attrs_define
class JointsRequest:
    """`POST /v1/arm/{side}/move_joints`.

    Attributes:
        joints_deg (list[float]): Target joint angles in degrees, seven joints.
        wait (bool): Block until the arm reports arrival (up to 30 s) rather than
            returning as soon as the command is accepted.
    """

    joints_deg: list[float]
    wait: bool
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        joints_deg = self.joints_deg

        wait = self.wait

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "joints_deg": joints_deg,
                "wait": wait,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        joints_deg = cast(list[float], d.pop("joints_deg"))

        wait = d.pop("wait")

        joints_request = cls(
            joints_deg=joints_deg,
            wait=wait,
        )

        joints_request.additional_properties = d
        return joints_request

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
