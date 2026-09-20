from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

T = TypeVar("T", bound="PixelFreezeReport")


@_attrs_define
class PixelFreezeReport:
    """The outcome of one frozen-window single-pixel placement.

    Attributes:
        delay_ms (int): Window speed the placement used, in milliseconds per index.
        stop_after_ms (float): Milliseconds from the `RUNNING` write to the `STOP` write, as the
            backend actually measured them.

            The diagnostic that says whether the pixel landed where it was aimed:
            a value more than half a step away from `target_ms` means the pixel
            may be on a neighbouring index.
        target_ms (float): Milliseconds after the `RUNNING` write at which the `STOP` was aimed,
            that is `(wire_index + 0.5) * delay_ms`.
        wire_index (int): Wire index the placement aimed at, `0..=17` as the controller counts
            the strip.
    """

    delay_ms: int
    stop_after_ms: float
    target_ms: float
    wire_index: int
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        delay_ms = self.delay_ms

        stop_after_ms = self.stop_after_ms

        target_ms = self.target_ms

        wire_index = self.wire_index

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "delay_ms": delay_ms,
                "stop_after_ms": stop_after_ms,
                "target_ms": target_ms,
                "wire_index": wire_index,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        delay_ms = d.pop("delay_ms")

        stop_after_ms = d.pop("stop_after_ms")

        target_ms = d.pop("target_ms")

        wire_index = d.pop("wire_index")

        pixel_freeze_report = cls(
            delay_ms=delay_ms,
            stop_after_ms=stop_after_ms,
            target_ms=target_ms,
            wire_index=wire_index,
        )

        pixel_freeze_report.additional_properties = d
        return pixel_freeze_report

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
