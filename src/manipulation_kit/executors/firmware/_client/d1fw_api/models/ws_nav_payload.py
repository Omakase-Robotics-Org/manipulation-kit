from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.nav_phase import NavPhase

T = TypeVar("T", bound="WsNavPayload")


@_attrs_define
class WsNavPayload:
    """The payload of a [`WsNavEvent`].

    Attributes:
        goal_id (int): A backend-assigned navigation goal identifier.
        message (str): Human-readable detail; empty when there is none.
        phase (NavPhase): The pinned navigation feedback vocabulary.
    """

    goal_id: int
    message: str
    phase: NavPhase
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        goal_id = self.goal_id

        message = self.message

        phase = self.phase.value

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "goal_id": goal_id,
                "message": message,
                "phase": phase,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        goal_id = d.pop("goal_id")

        message = d.pop("message")

        phase = NavPhase(d.pop("phase"))

        ws_nav_payload = cls(
            goal_id=goal_id,
            message=message,
            phase=phase,
        )

        ws_nav_payload.additional_properties = d
        return ws_nav_payload

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
