from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

T = TypeVar("T", bound="HandCapabilities")


@_attrs_define
class HandCapabilities:
    """What one hand can do, so a consumer can negotiate instead of assuming.

    Attributes:
        axis_names (list[str]): The axis names, in command order.
        features (list[str]): Capability tokens this driver implements. Names, not version numbers:
            a token is a promise that something works, tokens are added but never
            renamed or removed, and a consumer tests for the one it needs.
        max_command_hz (int): The vendor-rated command rate ceiling in hertz, covering commands AND
            polls together.
        model (str): Which model is fitted.
        num_axes (int): How many axes a command carries.
        presets (list[str]): Whether `open` and `fist` presets are offered.
        side (str): Which arm side it is mounted on.
        wire_max (int): The maximum raw value of [`HandUnit::Wire`] for this model.
    """

    axis_names: list[str]
    features: list[str]
    max_command_hz: int
    model: str
    num_axes: int
    presets: list[str]
    side: str
    wire_max: int
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        axis_names = self.axis_names

        features = self.features

        max_command_hz = self.max_command_hz

        model = self.model

        num_axes = self.num_axes

        presets = self.presets

        side = self.side

        wire_max = self.wire_max

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "axis_names": axis_names,
                "features": features,
                "max_command_hz": max_command_hz,
                "model": model,
                "num_axes": num_axes,
                "presets": presets,
                "side": side,
                "wire_max": wire_max,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        axis_names = cast(list[str], d.pop("axis_names"))

        features = cast(list[str], d.pop("features"))

        max_command_hz = d.pop("max_command_hz")

        model = d.pop("model")

        num_axes = d.pop("num_axes")

        presets = cast(list[str], d.pop("presets"))

        side = d.pop("side")

        wire_max = d.pop("wire_max")

        hand_capabilities = cls(
            axis_names=axis_names,
            features=features,
            max_command_hz=max_command_hz,
            model=model,
            num_axes=num_axes,
            presets=presets,
            side=side,
            wire_max=wire_max,
        )

        hand_capabilities.additional_properties = d
        return hand_capabilities

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
