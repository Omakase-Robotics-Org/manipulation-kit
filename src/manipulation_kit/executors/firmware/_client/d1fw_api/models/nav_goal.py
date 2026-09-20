from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

T = TypeVar("T", bound="NavGoal")


@_attrs_define
class NavGoal:
    """A chassis navigation goal.

    Attributes:
        point_id (str): Vendor point identifier.
        scene (str): Scene/map name.
        yaw (float): Desired yaw in radians.
    """

    point_id: str
    scene: str
    yaw: float
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        point_id = self.point_id

        scene = self.scene

        yaw = self.yaw

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "point_id": point_id,
                "scene": scene,
                "yaw": yaw,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        point_id = d.pop("point_id")

        scene = d.pop("scene")

        yaw = d.pop("yaw")

        nav_goal = cls(
            point_id=point_id,
            scene=scene,
            yaw=yaw,
        )

        nav_goal.additional_properties = d
        return nav_goal

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
