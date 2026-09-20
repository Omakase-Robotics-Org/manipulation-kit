from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.eye_target import EyeTarget
from ..types import UNSET, Unset

T = TypeVar("T", bound="EyesPixelRequest")


@_attrs_define
class EyesPixelRequest:
    """One frozen-window single-pixel placement request.

    `index` is the LED index as the eye controller counts the strip
    (`0..=17`), `target` names one eye (`left` or `right`; `both` is refused
    because the two strips do not run a `BOTH`-addressed window in phase), and
    `delay_ms` is the window speed in milliseconds per index, defaulting to
    the measured 100 ms.

        Attributes:
            b (int): Blue channel, `0..=255`.
            g (int): Green channel, `0..=255`.
            index (int): LED index on that strip, `0..=17`.
            r (int): Red channel, `0..=255`.
            target (EyeTarget): Selects which eye LED strip receives an effect.
            delay_ms (int | None | Unset): Window speed in milliseconds per index; omitted selects the measured
                100 ms default.
    """

    b: int
    g: int
    index: int
    r: int
    target: EyeTarget
    delay_ms: int | None | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        b = self.b

        g = self.g

        index = self.index

        r = self.r

        target = self.target.value

        delay_ms: int | None | Unset
        if isinstance(self.delay_ms, Unset):
            delay_ms = UNSET
        else:
            delay_ms = self.delay_ms

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "b": b,
                "g": g,
                "index": index,
                "r": r,
                "target": target,
            }
        )
        if delay_ms is not UNSET:
            field_dict["delay_ms"] = delay_ms

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        b = d.pop("b")

        g = d.pop("g")

        index = d.pop("index")

        r = d.pop("r")

        target = EyeTarget(d.pop("target"))

        def _parse_delay_ms(data: object) -> int | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(int | None | Unset, data)

        delay_ms = _parse_delay_ms(d.pop("delay_ms", UNSET))

        eyes_pixel_request = cls(
            b=b,
            g=g,
            index=index,
            r=r,
            target=target,
            delay_ms=delay_ms,
        )

        eyes_pixel_request.additional_properties = d
        return eyes_pixel_request

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
