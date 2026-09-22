from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from typing_extensions import Self

T = TypeVar("T", bound="ChassisCommonFileRead")


@_attrs_define
class ChassisCommonFileRead:
    """Exact basename within the explicit type; never a filesystem path.

    Attributes:
        file_name (str): 1..128 ASCII letters/digits/underscore/hyphen/dot; first character cannot be dot.
        file_type (str): Explicit manufacturer type component.
    """

    file_name: str
    file_type: str

    def to_dict(self) -> dict[str, Any]:
        file_name = self.file_name

        file_type = self.file_type

        field_dict: dict[str, Any] = {}

        field_dict.update(
            {
                "file_name": file_name,
                "file_type": file_type,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        file_name = d.pop("file_name")

        file_type = d.pop("file_type")

        chassis_common_file_read = cls(
            file_name=file_name,
            file_type=file_type,
        )

        return chassis_common_file_read
