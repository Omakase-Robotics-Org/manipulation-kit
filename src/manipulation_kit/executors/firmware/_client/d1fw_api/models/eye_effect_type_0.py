from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.eye_effect_type_0_effect import EyeEffectType0Effect

T = TypeVar("T", bound="EyeEffectType0")


@_attrs_define
class EyeEffectType0:
    """A breathing effect with an RGB color, step size, and delay.

    Attributes:
        b (int): Blue channel.
        delay_ms (int): Delay between steps in milliseconds.
        effect (EyeEffectType0Effect):
        g (int): Green channel.
        r (int): Red channel.
        step (int): Breathing step size.
    """

    b: int
    delay_ms: int
    effect: EyeEffectType0Effect
    g: int
    r: int
    step: int
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        b = self.b

        delay_ms = self.delay_ms

        effect = self.effect.value

        g = self.g

        r = self.r

        step = self.step

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "b": b,
                "delay_ms": delay_ms,
                "effect": effect,
                "g": g,
                "r": r,
                "step": step,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        b = d.pop("b")

        delay_ms = d.pop("delay_ms")

        effect = EyeEffectType0Effect(d.pop("effect"))

        g = d.pop("g")

        r = d.pop("r")

        step = d.pop("step")

        eye_effect_type_0 = cls(
            b=b,
            delay_ms=delay_ms,
            effect=effect,
            g=g,
            r=r,
            step=step,
        )

        eye_effect_type_0.additional_properties = d
        return eye_effect_type_0

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
