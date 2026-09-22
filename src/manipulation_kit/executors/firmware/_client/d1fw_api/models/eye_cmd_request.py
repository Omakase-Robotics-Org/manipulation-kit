from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.eye_cmd_request_effect import EyeCmdRequestEffect
from ..models.eye_target import EyeTarget
from ..types import UNSET, Unset

T = TypeVar("T", bound="EyeCmdRequest")


@_attrs_define
class EyeCmdRequest:
    """The advertised request body of `POST /v1/eyes/cmd`.

    This is a *flattened, advertise-only* view of [`d1fw_core::EyeCommand`],
    whose `#[serde(flatten)]` of the tagged [`d1fw_core::EyeEffect`] enum
    emitted an `allOf` over a `oneOf` — the shape `openapi-python-client`
    silently drops (it removed the `EyeCommand` model and skipped the
    endpoint). The daemon still deserializes the wire body into `EyeCommand`;
    nothing deserializes into this struct. `target` and `effect` are required,
    every colour/timing field is optional and applies only to the effects that
    name it.

        Attributes:
            effect (EyeCmdRequestEffect): Which eye LED effect to run; selects which of the other fields apply.
            target (EyeTarget): Selects which eye LED strip receives an effect.
            b (int | None | Unset): Blue channel, `0..=255`. Used by `breathing`, `running` and `solid`.
            delay_ms (int | None | Unset): Delay between steps in milliseconds. Used by `breathing` and `running`.
            g (int | None | Unset): Green channel, `0..=255`. Used by `breathing`, `running` and `solid`.
            r (int | None | Unset): Red channel, `0..=255`. Used by `breathing`, `running` and `solid`.
            step (int | None | Unset): Breathing step size or running-light window length. Used by
                `breathing` and `running`.
    """

    effect: EyeCmdRequestEffect
    target: EyeTarget
    b: int | None | Unset = UNSET
    delay_ms: int | None | Unset = UNSET
    g: int | None | Unset = UNSET
    r: int | None | Unset = UNSET
    step: int | None | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        effect = self.effect.value

        target = self.target.value

        b: int | None | Unset
        if isinstance(self.b, Unset):
            b = UNSET
        else:
            b = self.b

        delay_ms: int | None | Unset
        if isinstance(self.delay_ms, Unset):
            delay_ms = UNSET
        else:
            delay_ms = self.delay_ms

        g: int | None | Unset
        if isinstance(self.g, Unset):
            g = UNSET
        else:
            g = self.g

        r: int | None | Unset
        if isinstance(self.r, Unset):
            r = UNSET
        else:
            r = self.r

        step: int | None | Unset
        if isinstance(self.step, Unset):
            step = UNSET
        else:
            step = self.step

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "effect": effect,
                "target": target,
            }
        )
        if b is not UNSET:
            field_dict["b"] = b
        if delay_ms is not UNSET:
            field_dict["delay_ms"] = delay_ms
        if g is not UNSET:
            field_dict["g"] = g
        if r is not UNSET:
            field_dict["r"] = r
        if step is not UNSET:
            field_dict["step"] = step

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        effect = EyeCmdRequestEffect(d.pop("effect"))

        target = EyeTarget(d.pop("target"))

        def _parse_b(data: object) -> int | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(int | None | Unset, data)

        b = _parse_b(d.pop("b", UNSET))

        def _parse_delay_ms(data: object) -> int | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(int | None | Unset, data)

        delay_ms = _parse_delay_ms(d.pop("delay_ms", UNSET))

        def _parse_g(data: object) -> int | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(int | None | Unset, data)

        g = _parse_g(d.pop("g", UNSET))

        def _parse_r(data: object) -> int | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(int | None | Unset, data)

        r = _parse_r(d.pop("r", UNSET))

        def _parse_step(data: object) -> int | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(int | None | Unset, data)

        step = _parse_step(d.pop("step", UNSET))

        eye_cmd_request = cls(
            effect=effect,
            target=target,
            b=b,
            delay_ms=delay_ms,
            g=g,
            r=r,
            step=step,
        )

        eye_cmd_request.additional_properties = d
        return eye_cmd_request

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
