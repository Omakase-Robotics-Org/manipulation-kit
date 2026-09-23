from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.trajectory_guard import TrajectoryGuard
from ..models.trajectory_phase import TrajectoryPhase
from ..types import UNSET, Unset

T = TypeVar("T", bound="TrajectoryStatus")


@_attrs_define
class TrajectoryStatus:
    """Current or most recently completed job (bounded history: one job).

    Attributes:
        elapsed_ms (int): Actual elapsed playback time in milliseconds.
        id (int): Monotonic daemon-local job identifier.
        phase (TrajectoryPhase): The lifecycle phase of one trajectory job.

            `running` is the only non-terminal phase; the other three are final for
            that job identifier.
        guard (TrajectoryGuard | Unset): Which of the daemon's checks one trajectory is held to.

            The default is the whole guard. `speed_only` exists for taught gestures:
            a person moved the arm through every pose of such a trajectory by hand,
            so the operator decided whether it clears the body, and the guard's model
            margins must not veto what was demonstrated. The speed guard stays on
            for them; only the clearance ("range") guard may be switched off.
        message (None | str | Unset): Failure reason when phase is failed.
    """

    elapsed_ms: int
    id: int
    phase: TrajectoryPhase
    guard: TrajectoryGuard | Unset = UNSET
    message: None | str | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        elapsed_ms = self.elapsed_ms

        id = self.id

        phase = self.phase.value

        guard: str | Unset = UNSET
        if not isinstance(self.guard, Unset):
            guard = self.guard.value

        message: None | str | Unset
        if isinstance(self.message, Unset):
            message = UNSET
        else:
            message = self.message

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "elapsed_ms": elapsed_ms,
                "id": id,
                "phase": phase,
            }
        )
        if guard is not UNSET:
            field_dict["guard"] = guard
        if message is not UNSET:
            field_dict["message"] = message

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        elapsed_ms = d.pop("elapsed_ms")

        id = d.pop("id")

        phase = TrajectoryPhase(d.pop("phase"))

        _guard = d.pop("guard", UNSET)
        guard: TrajectoryGuard | Unset
        if isinstance(_guard, Unset):
            guard = UNSET
        else:
            guard = TrajectoryGuard(_guard)

        def _parse_message(data: object) -> None | str | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(None | str | Unset, data)

        message = _parse_message(d.pop("message", UNSET))

        trajectory_status = cls(
            elapsed_ms=elapsed_ms,
            id=id,
            phase=phase,
            guard=guard,
            message=message,
        )

        trajectory_status.additional_properties = d
        return trajectory_status

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
