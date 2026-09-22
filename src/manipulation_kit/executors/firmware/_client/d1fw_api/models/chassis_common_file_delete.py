from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from typing_extensions import Self

T = TypeVar("T", bound="ChassisCommonFileDelete")


@_attrs_define
class ChassisCommonFileDelete:
    """Delete one exact listed file after fresh current-content comparison.

    Attributes:
        expected_revision (str): Opaque selector+content revision from a present reading.
        file_name (str): Exact listed basename.
        file_type (str): Exact type component.
    """

    expected_revision: str
    file_name: str
    file_type: str

    def to_dict(self) -> dict[str, Any]:
        expected_revision = self.expected_revision

        file_name = self.file_name

        file_type = self.file_type

        field_dict: dict[str, Any] = {}

        field_dict.update(
            {
                "expected_revision": expected_revision,
                "file_name": file_name,
                "file_type": file_type,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        expected_revision = d.pop("expected_revision")

        file_name = d.pop("file_name")

        file_type = d.pop("file_type")

        chassis_common_file_delete = cls(
            expected_revision=expected_revision,
            file_name=file_name,
            file_type=file_type,
        )

        return chassis_common_file_delete
