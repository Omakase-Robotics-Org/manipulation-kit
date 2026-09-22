from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from typing_extensions import Self

T = TypeVar("T", bound="ChassisCommonFilesList")


@_attrs_define
class ChassisCommonFilesList:
    """One explicit manufacturer directory selector, not an inferred type inventory.

    Attributes:
        file_type (str): ASCII letters, digits, underscore or hyphen, 1..64 bytes.
    """

    file_type: str

    def to_dict(self) -> dict[str, Any]:
        file_type = self.file_type

        field_dict: dict[str, Any] = {}

        field_dict.update(
            {
                "file_type": file_type,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        file_type = d.pop("file_type")

        chassis_common_files_list = cls(
            file_type=file_type,
        )

        return chassis_common_files_list
