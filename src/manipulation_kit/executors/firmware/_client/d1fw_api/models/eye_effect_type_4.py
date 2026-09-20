from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.eye_effect_type_4_effect import EyeEffectType4Effect

T = TypeVar("T", bound="EyeEffectType4")


@_attrs_define
class EyeEffectType4:
    """Stop the selected strip's active effect.

    Attributes:
        effect (EyeEffectType4Effect):
    """

    effect: EyeEffectType4Effect
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        effect = self.effect.value

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "effect": effect,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        effect = EyeEffectType4Effect(d.pop("effect"))

        eye_effect_type_4 = cls(
            effect=effect,
        )

        eye_effect_type_4.additional_properties = d
        return eye_effect_type_4

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
