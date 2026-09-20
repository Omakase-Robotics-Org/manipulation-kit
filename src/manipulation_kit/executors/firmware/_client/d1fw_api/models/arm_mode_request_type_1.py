from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.arm_mode_request_type_1_mode import ArmModeRequestType1Mode

T = TypeVar("T", bound="ArmModeRequestType1")


@_attrs_define
class ArmModeRequestType1:
    """Enter position control with velocity and acceleration ratios.

    Attributes:
        acc_ratio (float): Acceleration ratio, as a backend-defined normalized ratio.
        mode (ArmModeRequestType1Mode):
        vel_ratio (float): Velocity ratio, as a backend-defined normalized ratio.
    """

    acc_ratio: float
    mode: ArmModeRequestType1Mode
    vel_ratio: float
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        acc_ratio = self.acc_ratio

        mode = self.mode.value

        vel_ratio = self.vel_ratio

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "acc_ratio": acc_ratio,
                "mode": mode,
                "vel_ratio": vel_ratio,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        acc_ratio = d.pop("acc_ratio")

        mode = ArmModeRequestType1Mode(d.pop("mode"))

        vel_ratio = d.pop("vel_ratio")

        arm_mode_request_type_1 = cls(
            acc_ratio=acc_ratio,
            mode=mode,
            vel_ratio=vel_ratio,
        )

        arm_mode_request_type_1.additional_properties = d
        return arm_mode_request_type_1

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
