from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

T = TypeVar("T", bound="SuggestedAction")


@_attrs_define
class SuggestedAction:
    """The one call an advisory suggests, as a frontend-independent verb and
    path.

    The REST route is what is published because it is the one form every
    frontend can be derived from: the WebSocket method for an arm's recover
    path is `arm.{side}.recover`, and the mapping between the two is already
    [`crate::API_MAP`].

        Attributes:
            method (str): The HTTP method, uppercase.
            path (str): The REST path, with any side already substituted.
    """

    method: str
    path: str
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        method = self.method

        path = self.path

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "method": method,
                "path": path,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        method = d.pop("method")

        path = d.pop("path")

        suggested_action = cls(
            method=method,
            path=path,
        )

        suggested_action.additional_properties = d
        return suggested_action

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
