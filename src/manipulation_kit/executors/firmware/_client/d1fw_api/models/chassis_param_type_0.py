from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.chassis_param_type_0_kind import ChassisParamType0Kind

T = TypeVar("T", bound="ChassisParamType0")


@_attrs_define
class ChassisParamType0:
    """The base's maximum driving speed, in metres per second, written as
    `max_linear_vel` into both of the chassis driver's parameter files.
    The vendor accepts up to and including 1.2.

        Attributes:
            kind (ChassisParamType0Kind):
            value (float): The base's maximum driving speed, in metres per second, written as
                `max_linear_vel` into both of the chassis driver's parameter files.
                The vendor accepts up to and including 1.2.
    """

    kind: ChassisParamType0Kind
    value: float
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        kind = self.kind.value

        value = self.value

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "kind": kind,
                "value": value,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        kind = ChassisParamType0Kind(d.pop("kind"))

        value = d.pop("value")

        chassis_param_type_0 = cls(
            kind=kind,
            value=value,
        )

        chassis_param_type_0.additional_properties = d
        return chassis_param_type_0

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
