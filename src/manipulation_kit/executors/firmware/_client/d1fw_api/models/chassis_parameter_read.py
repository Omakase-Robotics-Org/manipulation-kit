from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from typing_extensions import Self

T = TypeVar("T", bound="ChassisParameterRead")


@_attrs_define
class ChassisParameterRead:
    """Read a file that still exists in the vendor parameter inventory.

    Attributes:
        path (str): Exact path from parameter_files.
    """

    path: str

    def to_dict(self) -> dict[str, Any]:
        path = self.path

        field_dict: dict[str, Any] = {}

        field_dict.update(
            {
                "path": path,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        path = d.pop("path")

        chassis_parameter_read = cls(
            path=path,
        )

        return chassis_parameter_read
