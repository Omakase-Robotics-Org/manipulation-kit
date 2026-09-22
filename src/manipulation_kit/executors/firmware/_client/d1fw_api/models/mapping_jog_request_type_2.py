from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.mapping_jog_request_type_2_action import MappingJogRequestType2Action

T = TypeVar("T", bound="MappingJogRequestType2")


@_attrs_define
class MappingJogRequestType2:
    """End local intent and attempt explicit zero, even while killed or stale.

    Attributes:
        action (MappingJogRequestType2Action):
    """

    action: MappingJogRequestType2Action
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        action = self.action.value

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "action": action,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        action = MappingJogRequestType2Action(d.pop("action"))

        mapping_jog_request_type_2 = cls(
            action=action,
        )

        mapping_jog_request_type_2.additional_properties = d
        return mapping_jog_request_type_2

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
