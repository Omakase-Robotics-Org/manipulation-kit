from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..types import UNSET, Unset

T = TypeVar("T", bound="NeckCommand")


@_attrs_define
class NeckCommand:
    """A neck pose or pose-delta command.

    Attributes:
        pitch (float | None | Unset): Target or delta pitch in radians; positive is look-up.
        relative (bool | Unset): Interpret supplied axes as deltas when true.
        velocity (float | None | Unset): Requested speed in radians per second.  `None` selects the core default.
        yaw (float | None | Unset): Target or delta yaw in radians; positive is turn-right.
    """

    pitch: float | None | Unset = UNSET
    relative: bool | Unset = UNSET
    velocity: float | None | Unset = UNSET
    yaw: float | None | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        pitch: float | None | Unset
        if isinstance(self.pitch, Unset):
            pitch = UNSET
        else:
            pitch = self.pitch

        relative = self.relative

        velocity: float | None | Unset
        if isinstance(self.velocity, Unset):
            velocity = UNSET
        else:
            velocity = self.velocity

        yaw: float | None | Unset
        if isinstance(self.yaw, Unset):
            yaw = UNSET
        else:
            yaw = self.yaw

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update({})
        if pitch is not UNSET:
            field_dict["pitch"] = pitch
        if relative is not UNSET:
            field_dict["relative"] = relative
        if velocity is not UNSET:
            field_dict["velocity"] = velocity
        if yaw is not UNSET:
            field_dict["yaw"] = yaw

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)

        def _parse_pitch(data: object) -> float | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(float | None | Unset, data)

        pitch = _parse_pitch(d.pop("pitch", UNSET))

        relative = d.pop("relative", UNSET)

        def _parse_velocity(data: object) -> float | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(float | None | Unset, data)

        velocity = _parse_velocity(d.pop("velocity", UNSET))

        def _parse_yaw(data: object) -> float | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(float | None | Unset, data)

        yaw = _parse_yaw(d.pop("yaw", UNSET))

        neck_command = cls(
            pitch=pitch,
            relative=relative,
            velocity=velocity,
            yaw=yaw,
        )

        neck_command.additional_properties = d
        return neck_command

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
