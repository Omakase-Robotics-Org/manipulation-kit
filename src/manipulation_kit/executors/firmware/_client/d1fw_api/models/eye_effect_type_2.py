from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.eye_effect_type_2_effect import EyeEffectType2Effect

T = TypeVar("T", bound="EyeEffectType2")


@_attrs_define
class EyeEffectType2:
    """A solid RGB color.

    Attributes:
        b (int): Blue channel.
        effect (EyeEffectType2Effect):
        g (int): Green channel.
        r (int): Red channel.
    """

    b: int
    effect: EyeEffectType2Effect
    g: int
    r: int
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        b = self.b

        effect = self.effect.value

        g = self.g

        r = self.r

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "b": b,
                "effect": effect,
                "g": g,
                "r": r,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        b = d.pop("b")

        effect = EyeEffectType2Effect(d.pop("effect"))

        g = d.pop("g")

        r = d.pop("r")

        eye_effect_type_2 = cls(
            b=b,
            effect=effect,
            g=g,
            r=r,
        )

        eye_effect_type_2.additional_properties = d
        return eye_effect_type_2

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
