from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.trajectory_guard import TrajectoryGuard
from ..types import UNSET, Unset

if TYPE_CHECKING:
    from ..models.waypoint import Waypoint


T = TypeVar("T", bound="ArmTrajectoryStartBody")


@_attrs_define
class ArmTrajectoryStartBody:
    """
    Attributes:
        waypoints (list[Waypoint]): Two or more absolute-time waypoints, at most 10,000 and 120 seconds.
        guard (TrajectoryGuard | Unset): Which of the daemon's checks one trajectory is held to.

            The default is the whole guard. `speed_only` exists for taught gestures:
            a person moved the arm through every pose of such a trajectory by hand,
            so the operator decided whether it clears the body, and the guard's model
            margins must not veto what was demonstrated. The speed guard stays on
            for them; only the clearance ("range") guard may be switched off.
        holder (str | Unset): The arm lease holder issuing this command. Required only while somebody holds the lease:
            with no lease held the field is ignored, and with one held a request whose `holder` does not match is refused
            with 409.
    """

    waypoints: list[Waypoint]
    guard: TrajectoryGuard | Unset = UNSET
    holder: str | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        waypoints = []
        for waypoints_item_data in self.waypoints:
            waypoints_item = waypoints_item_data.to_dict()
            waypoints.append(waypoints_item)

        guard: str | Unset = UNSET
        if not isinstance(self.guard, Unset):
            guard = self.guard.value

        holder = self.holder

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "waypoints": waypoints,
            }
        )
        if guard is not UNSET:
            field_dict["guard"] = guard
        if holder is not UNSET:
            field_dict["holder"] = holder

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

        _guard = d.pop("guard", UNSET)
        guard: TrajectoryGuard | Unset
        if isinstance(_guard, Unset):
            guard = UNSET
        else:
            guard = TrajectoryGuard(_guard)

        holder = d.pop("holder", UNSET)

        arm_trajectory_start_body = cls(
            waypoints=waypoints,
            guard=guard,
            holder=holder,
        )

        arm_trajectory_start_body.additional_properties = d
        return arm_trajectory_start_body

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
