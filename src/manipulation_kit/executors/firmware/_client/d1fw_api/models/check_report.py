from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

T = TypeVar("T", bound="CheckReport")


@_attrs_define
class CheckReport:
    """Wire-level result of checking one or both arm poses.

    Attributes:
        valid (bool): Whether the supplied pose is accepted by the guard.
        violations (list[str]): Ordered, human-readable rejection reasons.
    """

    valid: bool
    violations: list[str]
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        valid = self.valid

        violations = self.violations

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "valid": valid,
                "violations": violations,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        valid = d.pop("valid")

        violations = cast(list[str], d.pop("violations"))

        check_report = cls(
            valid=valid,
            violations=violations,
        )

        check_report.additional_properties = d
        return check_report

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
