from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.hand_unit import HandUnit
from ..types import UNSET, Unset

T = TypeVar("T", bound="HandCommand")


@_attrs_define
class HandCommand:
    """One commanded hand pose.

    Attributes:
        axes (list[float]): One value per axis, in the model's own axis order (which
            [`HandCapabilities::axis_names`] publishes). The length must match
            [`HandCapabilities::num_axes`].
        unit (HandUnit | Unset): The unit a [`HandCommand`]'s values are in. See the module docs for why
            degrees are not among them.
    """

    axes: list[float]
    unit: HandUnit | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        axes = self.axes

        unit: str | Unset = UNSET
        if not isinstance(self.unit, Unset):
            unit = self.unit.value

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "axes": axes,
            }
        )
        if unit is not UNSET:
            field_dict["unit"] = unit

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        axes = cast(list[float], d.pop("axes"))

        _unit = d.pop("unit", UNSET)
        unit: HandUnit | Unset
        if isinstance(_unit, Unset):
            unit = UNSET
        else:
            unit = HandUnit(_unit)

        hand_command = cls(
            axes=axes,
            unit=unit,
        )

        hand_command.additional_properties = d
        return hand_command

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
