from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.arm_mode_request_type_3_mode import ArmModeRequestType3Mode
from ..types import UNSET, Unset

T = TypeVar("T", bound="ArmModeRequestType3")


@_attrs_define
class ArmModeRequestType3:
    """Enter Cartesian impedance control: the controller holds the commanded
    pose through a spring-damper acting in Cartesian space, so the end
    effector yields to external force and returns.

    Every field defaults to the reference implementation's own value; an
    omitted field is therefore a request for that default rather than for
    zero.

        Attributes:
            mode (ArmModeRequestType3Mode):
            acc_ratio (float | Unset): Acceleration ratio restored once the mode is live, as a normalized
                ratio.
            damping (list[float] | Unset): Cartesian damping, indexed exactly like `stiffness`.
            eef_rotation (list[float] | Unset): End-effector rotation parameters forwarded to the controller.
            eef_rotation_type (int | Unset): End-effector rotation type selector.
            stiffness (list[float] | Unset): Cartesian stiffness. Indices `0..=5` are the X, Y, Z, Rx, Ry, Rz
                stiffnesses; index `6` is the null-space stiffness for the arm's
                redundant seventh joint, not a seventh Cartesian axis.
            vel_ratio (float | Unset): Velocity ratio restored once the mode is live, as a normalized
                ratio. Mode entry itself always runs at a fixed reduced ratio.
    """

    mode: ArmModeRequestType3Mode
    acc_ratio: float | Unset = UNSET
    damping: list[float] | Unset = UNSET
    eef_rotation: list[float] | Unset = UNSET
    eef_rotation_type: int | Unset = UNSET
    stiffness: list[float] | Unset = UNSET
    vel_ratio: float | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        mode = self.mode.value

        acc_ratio = self.acc_ratio

        damping: list[float] | Unset = UNSET
        if not isinstance(self.damping, Unset):
            damping = self.damping

        eef_rotation: list[float] | Unset = UNSET
        if not isinstance(self.eef_rotation, Unset):
            eef_rotation = self.eef_rotation

        eef_rotation_type = self.eef_rotation_type

        stiffness: list[float] | Unset = UNSET
        if not isinstance(self.stiffness, Unset):
            stiffness = self.stiffness

        vel_ratio = self.vel_ratio

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "mode": mode,
            }
        )
        if acc_ratio is not UNSET:
            field_dict["acc_ratio"] = acc_ratio
        if damping is not UNSET:
            field_dict["damping"] = damping
        if eef_rotation is not UNSET:
            field_dict["eef_rotation"] = eef_rotation
        if eef_rotation_type is not UNSET:
            field_dict["eef_rotation_type"] = eef_rotation_type
        if stiffness is not UNSET:
            field_dict["stiffness"] = stiffness
        if vel_ratio is not UNSET:
            field_dict["vel_ratio"] = vel_ratio

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        mode = ArmModeRequestType3Mode(d.pop("mode"))

        acc_ratio = d.pop("acc_ratio", UNSET)

        damping = cast(list[float], d.pop("damping", UNSET))

        eef_rotation = cast(list[float], d.pop("eef_rotation", UNSET))

        eef_rotation_type = d.pop("eef_rotation_type", UNSET)

        stiffness = cast(list[float], d.pop("stiffness", UNSET))

        vel_ratio = d.pop("vel_ratio", UNSET)

        arm_mode_request_type_3 = cls(
            mode=mode,
            acc_ratio=acc_ratio,
            damping=damping,
            eef_rotation=eef_rotation,
            eef_rotation_type=eef_rotation_type,
            stiffness=stiffness,
            vel_ratio=vel_ratio,
        )

        arm_mode_request_type_3.additional_properties = d
        return arm_mode_request_type_3

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
