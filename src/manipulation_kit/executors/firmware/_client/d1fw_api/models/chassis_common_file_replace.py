from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from typing_extensions import Self

T = TypeVar("T", bound="ChassisCommonFileReplace")


@_attrs_define
class ChassisCommonFileReplace:
    """Replace an exact listed file after current raw-content revision comparison.

    Attributes:
        content_base64 (str): Canonical padded standard base64 of at most 512 KiB opaque bytes.
        expected_revision (str): Opaque selector+content revision from a present reading.
        file_name (str): Exact listed basename.
        file_type (str): Exact type component.
    """

    content_base64: str
    expected_revision: str
    file_name: str
    file_type: str

    def to_dict(self) -> dict[str, Any]:
        content_base64 = self.content_base64

        expected_revision = self.expected_revision

        file_name = self.file_name

        file_type = self.file_type

        field_dict: dict[str, Any] = {}

        field_dict.update(
            {
                "content_base64": content_base64,
                "expected_revision": expected_revision,
                "file_name": file_name,
                "file_type": file_type,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        content_base64 = d.pop("content_base64")

        expected_revision = d.pop("expected_revision")

        file_name = d.pop("file_name")

        file_type = d.pop("file_type")

        chassis_common_file_replace = cls(
            content_base64=content_base64,
            expected_revision=expected_revision,
            file_name=file_name,
            file_type=file_type,
        )

        return chassis_common_file_replace
