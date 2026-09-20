from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

T = TypeVar("T", bound="ClampReport")


@_attrs_define
class ClampReport:
    """Wire-level result of clamping one arm pose to its URDF limits.

    Attributes:
        changed (bool): Whether any joint differs exactly from its supplied value.
        clamped_deg (list[float]): The seven clamped joint angles in degrees, or an empty vector for an invalid request.
    """

    changed: bool
    clamped_deg: list[float]
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        changed = self.changed

        clamped_deg = self.clamped_deg

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "changed": changed,
                "clamped_deg": clamped_deg,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        changed = d.pop("changed")

        clamped_deg = cast(list[float], d.pop("clamped_deg"))

        clamp_report = cls(
            changed=changed,
            clamped_deg=clamped_deg,
        )

        clamp_report.additional_properties = d
        return clamp_report

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
