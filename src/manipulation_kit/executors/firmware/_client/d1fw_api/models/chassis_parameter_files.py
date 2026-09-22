from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

if TYPE_CHECKING:
    from ..models.chassis_parameter_file import ChassisParameterFile


T = TypeVar("T", bound="ChassisParameterFiles")


@_attrs_define
class ChassisParameterFiles:
    """Complete bounded discovery; an incomplete traversal returns an error.

    Attributes:
        files (list[ChassisParameterFile]): Existing configuration files, excluding logs and temporary artifacts.
    """

    files: list[ChassisParameterFile]
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        files = []
        for files_item_data in self.files:
            files_item = files_item_data.to_dict()
            files.append(files_item)

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "files": files,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.chassis_parameter_file import (
            ChassisParameterFile,
        )

        d = dict(src_dict)
        files = []
        _files = d.pop("files")
        for files_item_data in _files:
            files_item = ChassisParameterFile.from_dict(files_item_data)

            files.append(files_item)

        chassis_parameter_files = cls(
            files=files,
        )

        chassis_parameter_files.additional_properties = d
        return chassis_parameter_files

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
