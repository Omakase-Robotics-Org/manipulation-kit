from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.chassis_param_type_5_kind import ChassisParamType5Kind

T = TypeVar("T", bound="ChassisParamType5")


@_attrs_define
class ChassisParamType5:
    """The base's footprint polygon: four `[x, y]` corners, in metres,
    written as `footprint` into `navigator/costmap_params.yaml` and as
    `vertices` into `navigator/teb_local_planner_params.yaml`.

        Attributes:
            kind (ChassisParamType5Kind):
            value (list[list[float]]): The base's footprint polygon: four `[x, y]` corners, in metres,
                written as `footprint` into `navigator/costmap_params.yaml` and as
                `vertices` into `navigator/teb_local_planner_params.yaml`.
    """

    kind: ChassisParamType5Kind
    value: list[list[float]]
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        kind = self.kind.value

        value = []
        for value_item_data in self.value:
            value_item = value_item_data

            value.append(value_item)

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
        kind = ChassisParamType5Kind(d.pop("kind"))

        value = []
        _value = d.pop("value")
        for value_item_data in _value:
            value_item = cast(list[float], value_item_data)

            value.append(value_item)

        chassis_param_type_5 = cls(
            kind=kind,
            value=value,
        )

        chassis_param_type_5.additional_properties = d
        return chassis_param_type_5

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
