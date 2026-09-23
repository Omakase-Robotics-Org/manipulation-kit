from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

if TYPE_CHECKING:
    from ..models.waypoint import Waypoint


T = TypeVar("T", bound="TrajectoryRequest")


@_attrs_define
class TrajectoryRequest:
    """A bounded trajectory upload. Mode and tool selection remain explicit verbs.

    Attributes:
        waypoints (list[Waypoint]): Two or more absolute-time waypoints, at most 10,000 and 120 seconds.
    """

    waypoints: list[Waypoint]
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        waypoints = []
        for waypoints_item_data in self.waypoints:
            waypoints_item = waypoints_item_data.to_dict()
            waypoints.append(waypoints_item)

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "waypoints": waypoints,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.waypoint import Waypoint

        d = dict(src_dict)
        waypoints = []
        _waypoints = d.pop("waypoints")
        for waypoints_item_data in _waypoints:
            waypoints_item = Waypoint.from_dict(waypoints_item_data)

            waypoints.append(waypoints_item)

        trajectory_request = cls(
            waypoints=waypoints,
        )

        trajectory_request.additional_properties = d
        return trajectory_request

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
