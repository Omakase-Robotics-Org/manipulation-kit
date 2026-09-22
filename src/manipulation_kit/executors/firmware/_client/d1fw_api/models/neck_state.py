from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

T = TypeVar("T", bound="NeckState")


@_attrs_define
class NeckState:
    """Neck feedback in the logical robot frame.

    Attributes:
        enabled (bool): Whether the neck motors are enabled.
        moving (bool): Whether either neck axis is moving.

            The backend's own answer, taken from the same feedback frames as the
            velocities beside it: an axis is moving when its measured velocity
            exceeds what the motor's feedback can express, which is the wire's
            resolution rather than a chosen threshold. A neck creeping below that
            resolution reads `false`; nothing on this wire can tell it apart from
            a standing motor. See [`crate::Motion`], which folds this into the
            whole-robot answer.
        pitch (float): Current pitch in radians.
        pitch_torque (float): Measured pitch torque in newton-metres.
        pitch_velocity (float): Current pitch velocity in radians per second.
        yaw (float): Current yaw in radians.
        yaw_torque (float): Measured yaw torque in newton-metres.
        yaw_velocity (float): Current yaw velocity in radians per second.
    """

    enabled: bool
    moving: bool
    pitch: float
    pitch_torque: float
    pitch_velocity: float
    yaw: float
    yaw_torque: float
    yaw_velocity: float
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        enabled = self.enabled

        moving = self.moving

        pitch = self.pitch

        pitch_torque = self.pitch_torque

        pitch_velocity = self.pitch_velocity

        yaw = self.yaw

        yaw_torque = self.yaw_torque

        yaw_velocity = self.yaw_velocity

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "enabled": enabled,
                "moving": moving,
                "pitch": pitch,
                "pitch_torque": pitch_torque,
                "pitch_velocity": pitch_velocity,
                "yaw": yaw,
                "yaw_torque": yaw_torque,
                "yaw_velocity": yaw_velocity,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        enabled = d.pop("enabled")

        moving = d.pop("moving")

        pitch = d.pop("pitch")

        pitch_torque = d.pop("pitch_torque")

        pitch_velocity = d.pop("pitch_velocity")

        yaw = d.pop("yaw")

        yaw_torque = d.pop("yaw_torque")

        yaw_velocity = d.pop("yaw_velocity")

        neck_state = cls(
            enabled=enabled,
            moving=moving,
            pitch=pitch,
            pitch_torque=pitch_torque,
            pitch_velocity=pitch_velocity,
            yaw=yaw,
            yaw_torque=yaw_torque,
            yaw_velocity=yaw_velocity,
        )

        neck_state.additional_properties = d
        return neck_state

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
