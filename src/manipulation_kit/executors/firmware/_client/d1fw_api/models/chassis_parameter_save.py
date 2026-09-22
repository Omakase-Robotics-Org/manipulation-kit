from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from typing_extensions import Self

T = TypeVar("T", bound="ChassisParameterSave")


@_attrs_define
class ChassisParameterSave:
    """Conditional text save; no creation or arbitrary filesystem access.
    REST limits the complete JSON request to one MiB, including expected_text,
    text, path and JSON escaping. Each text can therefore be below one MiB
    while their combined encoded request is still rejected by the transport.

        Attributes:
            expected_text (str): Original text; compared again immediately before writing.
            path (str): Exact inventory path.
            text (str): Replacement text, at most one MiB and without NUL characters.
    """

    expected_text: str
    path: str
    text: str

    def to_dict(self) -> dict[str, Any]:
        expected_text = self.expected_text

        path = self.path

        text = self.text

        field_dict: dict[str, Any] = {}

        field_dict.update(
            {
                "expected_text": expected_text,
                "path": path,
                "text": text,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        expected_text = d.pop("expected_text")

        path = d.pop("path")

        text = d.pop("text")

        chassis_parameter_save = cls(
            expected_text=expected_text,
            path=path,
            text=text,
        )

        return chassis_parameter_save
