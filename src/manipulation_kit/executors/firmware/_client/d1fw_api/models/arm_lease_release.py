from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

T = TypeVar("T", bound="ArmLeaseRelease")


@_attrs_define
class ArmLeaseRelease:
    """`DELETE /v1/arm/lease`: hand the arms back.

    Attributes:
        holder (str): The holder releasing the lease. Releasing somebody else's lease is
            refused with 409: a release is a consumer yielding, never a takeover.
    """

    holder: str
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        holder = self.holder

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "holder": holder,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        holder = d.pop("holder")

        arm_lease_release = cls(
            holder=holder,
        )

        arm_lease_release.additional_properties = d
        return arm_lease_release

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
