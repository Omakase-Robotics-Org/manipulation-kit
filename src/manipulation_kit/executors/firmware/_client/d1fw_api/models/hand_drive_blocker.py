from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.hand_drive_blocker_kind import HandDriveBlockerKind

T = TypeVar("T", bound="HandDriveBlocker")


@_attrs_define
class HandDriveBlocker:
    """One unmet precondition of hand-driving, with the sentence that says what
    to do about it.

        Attributes:
            kind (HandDriveBlockerKind): One precondition of hand-driving the mobile base that is not currently
                met.
            reason (str): What it means and what clears it, in one sentence.
    """

    kind: HandDriveBlockerKind
    reason: str
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        kind = self.kind.value

        reason = self.reason

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "kind": kind,
                "reason": reason,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        kind = HandDriveBlockerKind(d.pop("kind"))

        reason = d.pop("reason")

        hand_drive_blocker = cls(
            kind=kind,
            reason=reason,
        )

        hand_drive_blocker.additional_properties = d
        return hand_drive_blocker

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
