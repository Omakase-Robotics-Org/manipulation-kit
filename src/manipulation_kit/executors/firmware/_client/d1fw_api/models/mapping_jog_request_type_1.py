from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.mapping_jog_request_type_1_action import MappingJogRequestType1Action

T = TypeVar("T", bound="MappingJogRequestType1")


@_attrs_define
class MappingJogRequestType1:
    """Refresh only the identified live lease.

    Attributes:
        action (MappingJogRequestType1Action):
        lease_id (str): Exact opaque lease identity from start acknowledgment.
    """

    action: MappingJogRequestType1Action
    lease_id: str
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        action = self.action.value

        lease_id = self.lease_id

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "action": action,
                "lease_id": lease_id,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        action = MappingJogRequestType1Action(d.pop("action"))

        lease_id = d.pop("lease_id")

        mapping_jog_request_type_1 = cls(
            action=action,
            lease_id=lease_id,
        )

        mapping_jog_request_type_1.additional_properties = d
        return mapping_jog_request_type_1

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
