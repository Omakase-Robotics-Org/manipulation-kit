from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.chassis_param_type_4_kind import ChassisParamType4Kind

T = TypeVar("T", bound="ChassisParamType4")


@_attrs_define
class ChassisParamType4:
    """The charging-dock stop distance, in metres, written as
    `lidar_docking.dist_stop` into `docking/lidar_dock.yaml`. The vendor
    accepts the inclusive range 0.225 to 0.425.

        Attributes:
            kind (ChassisParamType4Kind):
            value (float): The charging-dock stop distance, in metres, written as
                `lidar_docking.dist_stop` into `docking/lidar_dock.yaml`. The vendor
                accepts the inclusive range 0.225 to 0.425.
    """

    kind: ChassisParamType4Kind
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
        kind = ChassisParamType4Kind(d.pop("kind"))

        value = d.pop("value")

        chassis_param_type_4 = cls(
            kind=kind,
            value=value,
        )

        chassis_param_type_4.additional_properties = d
        return chassis_param_type_4

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
