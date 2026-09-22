from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

T = TypeVar("T", bound="ChassisParameterFile")


@_attrs_define
class ChassisParameterFile:
    """An existing editable parameter file from bounded vendor discovery.

    Attributes:
        format_ (str): Filename extension; does not establish parameter semantics.
        name (str): Basename for display.
        path (str): Exact vendor-discovered relative path.
    """

    format_: str
    name: str
    path: str
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        format_ = self.format_

        name = self.name

        path = self.path

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "format": format_,
                "name": name,
                "path": path,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        format_ = d.pop("format")

        name = d.pop("name")

        path = d.pop("path")

        chassis_parameter_file = cls(
            format_=format_,
            name=name,
            path=path,
        )

        chassis_parameter_file.additional_properties = d
        return chassis_parameter_file

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
