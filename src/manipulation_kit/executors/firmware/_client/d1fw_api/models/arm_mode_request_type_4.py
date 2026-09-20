from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.arm_mode_request_type_4_mode import ArmModeRequestType4Mode
from ..types import UNSET, Unset

T = TypeVar("T", bound="ArmModeRequestType4")


@_attrs_define
class ArmModeRequestType4:
    """Enter force/compliance control: the controller yields along the
    enabled Cartesian directions, optionally pressing with a target force.

    Every field defaults to the reference implementation's own value.

        Attributes:
            mode (ArmModeRequestType4Mode):
            acc_ratio (float | Unset): Acceleration ratio restored once the mode is live, as a normalized
                ratio.
            adjustment_limit_mm (float | Unset): How far, in millimetres, the controller may displace the end
                effector in order to comply. This is the softness knob: a larger
                value makes the arm more backdrivable. Must be positive.
            anchor_command_pose (bool | Unset): Write the measured pose into the controller's commanded pose
                immediately before the transition. On by default, and unsafe to
                turn off: the commanded pose persists in the controller, so
                without anchoring, engaging torque tells the arm to drive to
                whatever pose was left there.
            control_params (list[float] | Unset): Force-loop gains and limits. All zeros selects the controller's
                built-in defaults, which is the supported configuration.
            force_direction (list[float] | Unset): Which Cartesian directions (X, Y, Z, Rx, Ry, Rz) the force loop
                acts on; `1` enables a direction and `0` disables it. At least one
                direction must be enabled.
            force_type (int | Unset): Force-control strategy selector; the reference implementation uses
                zero.
            target_force (float | Unset): Target contact force in newtons, and the field to leave at zero
                unless the end effector is against something that can react.

                This is an active command, not a sensitivity threshold: any
                positive value makes the controller push along `force_direction`
                until it measures that force, which in free space never happens,
                so the arm creeps continuously. Zero means pure compliance — hold
                the pose, yield to contact. Must not be negative.
            vel_ratio (float | Unset): Velocity ratio restored once the mode is live, as a normalized
                ratio.
            zero_wrist_roll (bool | Unset): Command the wrist-roll joint (J7) to zero as part of the entry
                anchor, rather than freezing it at whatever angle the arm arrived
                in.

                The joint is equally stiff either way — the default direction set
                enables no rotation — but this changes the angle it is stiff at.
                It commands a motion at entry, so it is off by default.
    """

    mode: ArmModeRequestType4Mode
    acc_ratio: float | Unset = UNSET
    adjustment_limit_mm: float | Unset = UNSET
    anchor_command_pose: bool | Unset = UNSET
    control_params: list[float] | Unset = UNSET
    force_direction: list[float] | Unset = UNSET
    force_type: int | Unset = UNSET
    target_force: float | Unset = UNSET
    vel_ratio: float | Unset = UNSET
    zero_wrist_roll: bool | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        mode = self.mode.value

        acc_ratio = self.acc_ratio

        adjustment_limit_mm = self.adjustment_limit_mm

        anchor_command_pose = self.anchor_command_pose

        control_params: list[float] | Unset = UNSET
        if not isinstance(self.control_params, Unset):
            control_params = self.control_params

        force_direction: list[float] | Unset = UNSET
        if not isinstance(self.force_direction, Unset):
            force_direction = self.force_direction

        force_type = self.force_type

        target_force = self.target_force

        vel_ratio = self.vel_ratio

        zero_wrist_roll = self.zero_wrist_roll

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "mode": mode,
            }
        )
        if acc_ratio is not UNSET:
            field_dict["acc_ratio"] = acc_ratio
        if adjustment_limit_mm is not UNSET:
            field_dict["adjustment_limit_mm"] = adjustment_limit_mm
        if anchor_command_pose is not UNSET:
            field_dict["anchor_command_pose"] = anchor_command_pose
        if control_params is not UNSET:
            field_dict["control_params"] = control_params
        if force_direction is not UNSET:
            field_dict["force_direction"] = force_direction
        if force_type is not UNSET:
            field_dict["force_type"] = force_type
        if target_force is not UNSET:
            field_dict["target_force"] = target_force
        if vel_ratio is not UNSET:
            field_dict["vel_ratio"] = vel_ratio
        if zero_wrist_roll is not UNSET:
            field_dict["zero_wrist_roll"] = zero_wrist_roll

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        mode = ArmModeRequestType4Mode(d.pop("mode"))

        acc_ratio = d.pop("acc_ratio", UNSET)

        adjustment_limit_mm = d.pop("adjustment_limit_mm", UNSET)

        anchor_command_pose = d.pop("anchor_command_pose", UNSET)

        control_params = cast(list[float], d.pop("control_params", UNSET))

        force_direction = cast(list[float], d.pop("force_direction", UNSET))

        force_type = d.pop("force_type", UNSET)

        target_force = d.pop("target_force", UNSET)

        vel_ratio = d.pop("vel_ratio", UNSET)

        zero_wrist_roll = d.pop("zero_wrist_roll", UNSET)

        arm_mode_request_type_4 = cls(
            mode=mode,
            acc_ratio=acc_ratio,
            adjustment_limit_mm=adjustment_limit_mm,
            anchor_command_pose=anchor_command_pose,
            control_params=control_params,
            force_direction=force_direction,
            force_type=force_type,
            target_force=target_force,
            vel_ratio=vel_ratio,
            zero_wrist_roll=zero_wrist_roll,
        )

        arm_mode_request_type_4.additional_properties = d
        return arm_mode_request_type_4

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
