from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

T = TypeVar("T", bound="ControlStep")


@_attrs_define
class ControlStep:
    """One thing the daemon did while carrying out a control verb.

    Attributes:
        action (str): What was done, as a stable word: `close_navigation`, `settle`,
            `enter_remote_control`, `leave_remote_control`, `start_navigation`,
            `engage_motors`, `release_motors`, `read_back`.
        detail (str): The detail of that step: the vendor call that was sent, how long a
            settle waited, or what the read-back saw.
    """

    action: str
    detail: str
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        action = self.action

        detail = self.detail

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "action": action,
                "detail": detail,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        action = d.pop("action")

        detail = d.pop("detail")

        control_step = cls(
            action=action,
            detail=detail,
        )

        control_step.additional_properties = d
        return control_step

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
