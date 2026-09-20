from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

T = TypeVar("T", bound="KernelModule")


@_attrs_define
class KernelModule:
    """One loaded kernel module of interest (from `lsmod`).

    Attributes:
        name (str): Module name (e.g. `ec_master`).
        size (int): Module size in bytes as reported by `lsmod`.
        used_by (list[str]): Names of modules that depend on this one (the `lsmod` "Used by" list).
    """

    name: str
    size: int
    used_by: list[str]
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        name = self.name

        size = self.size

        used_by = self.used_by

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "name": name,
                "size": size,
                "used_by": used_by,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        name = d.pop("name")

        size = d.pop("size")

        used_by = cast(list[str], d.pop("used_by"))

        kernel_module = cls(
            name=name,
            size=size,
            used_by=used_by,
        )

        kernel_module.additional_properties = d
        return kernel_module

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
