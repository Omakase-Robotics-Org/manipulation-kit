from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.arm_mode_request_type_2_mode import ArmModeRequestType2Mode

T = TypeVar("T", bound="ArmModeRequestType2")


@_attrs_define
class ArmModeRequestType2:
    """Joint impedance in the controller's TORQ state (ImpType=1).
    Joint targets remain commandable. Gains are explicit so a caller
    supplies the mounted robot's chosen joint stiffness and damping.

        Attributes:
            acc_ratio (float): Normalized acceleration ratio restored after anchored entry.
            damping (list[float]): Joint damping in N m/(degree/s), seven joints.
            mode (ArmModeRequestType2Mode):
            stiffness (list[float]): Joint stiffness in N m/degree, seven joints.
            vel_ratio (float): Normalized velocity ratio restored after anchored entry.
    """

    acc_ratio: float
    damping: list[float]
    mode: ArmModeRequestType2Mode
    stiffness: list[float]
    vel_ratio: float
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        acc_ratio = self.acc_ratio

        damping = self.damping

        mode = self.mode.value

        stiffness = self.stiffness

        vel_ratio = self.vel_ratio

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "acc_ratio": acc_ratio,
                "damping": damping,
                "mode": mode,
                "stiffness": stiffness,
                "vel_ratio": vel_ratio,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        acc_ratio = d.pop("acc_ratio")

        damping = cast(list[float], d.pop("damping"))

        mode = ArmModeRequestType2Mode(d.pop("mode"))

        stiffness = cast(list[float], d.pop("stiffness"))

        vel_ratio = d.pop("vel_ratio")

        arm_mode_request_type_2 = cls(
            acc_ratio=acc_ratio,
            damping=damping,
            mode=mode,
            stiffness=stiffness,
            vel_ratio=vel_ratio,
        )

        arm_mode_request_type_2.additional_properties = d
        return arm_mode_request_type_2

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
