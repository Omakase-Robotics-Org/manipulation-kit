from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from typing_extensions import Self

T = TypeVar("T", bound="ChassisSettingsSave")


@_attrs_define
class ChassisSettingsSave:
    """Conditional replacement; omitted object keys are retained recursively.
    REST limits the complete JSON request, including both snapshots, to one MiB.

        Attributes:
            expected (Any): Original complete record; a changed vendor record refuses the write.
            value (Any): Edited record. Arrays replace in full; object keys cannot be deleted.
    """

    expected: Any
    value: Any

    def to_dict(self) -> dict[str, Any]:
        expected = self.expected

        value = self.value

        field_dict: dict[str, Any] = {}

        field_dict.update(
            {
                "expected": expected,
                "value": value,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        expected = d.pop("expected")

        value = d.pop("value")

        chassis_settings_save = cls(
            expected=expected,
            value=value,
        )

        return chassis_settings_save
