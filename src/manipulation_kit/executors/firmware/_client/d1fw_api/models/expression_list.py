from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

T = TypeVar("T", bound="ExpressionList")


@_attrs_define
class ExpressionList:
    """The expression names and raw catalog types in index order.

    Attributes:
        names (list[str]): Catalog names.
        types (list[str]): Catalog type strings (`timed` or `scalar`).
    """

    names: list[str]
    types: list[str]
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        names = self.names

        types = self.types

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "names": names,
                "types": types,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        names = cast(list[str], d.pop("names"))

        types = cast(list[str], d.pop("types"))

        expression_list = cls(
            names=names,
            types=types,
        )

        expression_list.additional_properties = d
        return expression_list

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
