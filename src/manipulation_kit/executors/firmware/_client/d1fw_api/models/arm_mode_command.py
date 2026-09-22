from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.arm_mode_command_mode import ArmModeCommandMode
from ..types import UNSET, Unset

T = TypeVar("T", bound="ArmModeCommand")


@_attrs_define
class ArmModeCommand:
    """The advertised request body of `POST /v1/arm/{side}/mode`.

    This is a *flattened, advertise-only* view of the tagged
    [`d1fw_core::ArmModeRequest`] enum with the arm lease's optional `holder`
    folded in as a top-level field. The daemon still deserializes the wire
    body into `ArmModeRequest` and reads `holder` off it separately (see
    `d1fw_front_rest::lease`); nothing deserializes into this struct. It exists
    only so the published document describes the body as one flat object.

    `openapi-python-client` silently skips any endpoint whose request body is
    an `allOf` containing a `oneOf`, which is exactly what the tagged enum plus
    a composed-in `holder` used to emit — so `POST /v1/arm/{side}/mode`, the
    most-used arm verb, was missing from every generated Python client. A
    single flat object with a `mode` string discriminator and every per-variant
    parameter optional generates cleanly. Which fields are required for a given
    `mode`, and their defaults, are documented on [`d1fw_core::ArmModeRequest`]
    and enforced by the daemon, not by this looser schema.

        Attributes:
            mode (ArmModeCommandMode): Which arm control mode to enter; selects which of the other fields apply.
            acc_ratio (float | None | Unset): Acceleration ratio, as a backend-defined normalized ratio
                (`0.0..=1.0`). Used by the same four modes as `vel_ratio`.
            adjustment_limit_mm (float | None | Unset): `force_compliance`: how far, in millimetres, the controller may
                displace the end effector to comply. Must be positive.
            anchor_command_pose (bool | None | Unset): `force_compliance`: write the measured pose into the controller's
                commanded pose before the transition. On by default, unsafe to disable.
            control_params (list[float] | None | Unset): `force_compliance`: force-loop gains and limits; all zeros selects
                the
                controller's built-in defaults.
            damping (list[float] | None | Unset): Joint (`torque`) or Cartesian (`cartesian_impedance`) damping, seven
                entries. Omitted requests the mode's own default.
            eef_rotation (list[float] | None | Unset): `cartesian_impedance`: end-effector rotation parameters forwarded to
                the controller.
            eef_rotation_type (int | None | Unset): `cartesian_impedance`: end-effector rotation type selector.
            force_direction (list[float] | None | Unset): `force_compliance`: which Cartesian directions (X, Y, Z, Rx, Ry,
                Rz)
                the force loop acts on; `1` enables a direction, `0` disables it.
            force_type (int | None | Unset): `force_compliance`: force-control strategy selector.
            holder (None | str | Unset): The arm lease holder issuing this command. Required only while somebody
                holds the lease: with no lease held the field is ignored, and with one
                held a request whose `holder` does not match is refused with 409.
            stiffness (list[float] | None | Unset): Joint (`torque`) or Cartesian (`cartesian_impedance`) stiffness, seven
                entries. Omitted requests the mode's own default.
            target_force (float | None | Unset): `force_compliance`: target contact force in newtons. Must not be
                negative; any positive value is an active push, not a threshold.
            vel_ratio (float | None | Unset): Velocity ratio, as a backend-defined normalized ratio (`0.0..=1.0`).
                Used by `position`, `torque`, `cartesian_impedance` and
                `force_compliance`.
            zero_wrist_roll (bool | None | Unset): `force_compliance`: command the wrist-roll joint (J7) to zero as part
                of the entry anchor rather than freezing it in place.
    """

    mode: ArmModeCommandMode
    acc_ratio: float | None | Unset = UNSET
    adjustment_limit_mm: float | None | Unset = UNSET
    anchor_command_pose: bool | None | Unset = UNSET
    control_params: list[float] | None | Unset = UNSET
    damping: list[float] | None | Unset = UNSET
    eef_rotation: list[float] | None | Unset = UNSET
    eef_rotation_type: int | None | Unset = UNSET
    force_direction: list[float] | None | Unset = UNSET
    force_type: int | None | Unset = UNSET
    holder: None | str | Unset = UNSET
    stiffness: list[float] | None | Unset = UNSET
    target_force: float | None | Unset = UNSET
    vel_ratio: float | None | Unset = UNSET
    zero_wrist_roll: bool | None | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        mode = self.mode.value

        acc_ratio: float | None | Unset
        if isinstance(self.acc_ratio, Unset):
            acc_ratio = UNSET
        else:
            acc_ratio = self.acc_ratio

        adjustment_limit_mm: float | None | Unset
        if isinstance(self.adjustment_limit_mm, Unset):
            adjustment_limit_mm = UNSET
        else:
            adjustment_limit_mm = self.adjustment_limit_mm

        anchor_command_pose: bool | None | Unset
        if isinstance(self.anchor_command_pose, Unset):
            anchor_command_pose = UNSET
        else:
            anchor_command_pose = self.anchor_command_pose

        control_params: list[float] | None | Unset
        if isinstance(self.control_params, Unset):
            control_params = UNSET
        elif isinstance(self.control_params, list):
            control_params = self.control_params

        else:
            control_params = self.control_params

        damping: list[float] | None | Unset
        if isinstance(self.damping, Unset):
            damping = UNSET
        elif isinstance(self.damping, list):
            damping = self.damping

        else:
            damping = self.damping

        eef_rotation: list[float] | None | Unset
        if isinstance(self.eef_rotation, Unset):
            eef_rotation = UNSET
        elif isinstance(self.eef_rotation, list):
            eef_rotation = self.eef_rotation

        else:
            eef_rotation = self.eef_rotation

        eef_rotation_type: int | None | Unset
        if isinstance(self.eef_rotation_type, Unset):
            eef_rotation_type = UNSET
        else:
            eef_rotation_type = self.eef_rotation_type

        force_direction: list[float] | None | Unset
        if isinstance(self.force_direction, Unset):
            force_direction = UNSET
        elif isinstance(self.force_direction, list):
            force_direction = self.force_direction

        else:
            force_direction = self.force_direction

        force_type: int | None | Unset
        if isinstance(self.force_type, Unset):
            force_type = UNSET
        else:
            force_type = self.force_type

        holder: None | str | Unset
        if isinstance(self.holder, Unset):
            holder = UNSET
        else:
            holder = self.holder

        stiffness: list[float] | None | Unset
        if isinstance(self.stiffness, Unset):
            stiffness = UNSET
        elif isinstance(self.stiffness, list):
            stiffness = self.stiffness

        else:
            stiffness = self.stiffness

        target_force: float | None | Unset
        if isinstance(self.target_force, Unset):
            target_force = UNSET
        else:
            target_force = self.target_force

        vel_ratio: float | None | Unset
        if isinstance(self.vel_ratio, Unset):
            vel_ratio = UNSET
        else:
            vel_ratio = self.vel_ratio

        zero_wrist_roll: bool | None | Unset
        if isinstance(self.zero_wrist_roll, Unset):
            zero_wrist_roll = UNSET
        else:
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
        if damping is not UNSET:
            field_dict["damping"] = damping
        if eef_rotation is not UNSET:
            field_dict["eef_rotation"] = eef_rotation
        if eef_rotation_type is not UNSET:
            field_dict["eef_rotation_type"] = eef_rotation_type
        if force_direction is not UNSET:
            field_dict["force_direction"] = force_direction
        if force_type is not UNSET:
            field_dict["force_type"] = force_type
        if holder is not UNSET:
            field_dict["holder"] = holder
        if stiffness is not UNSET:
            field_dict["stiffness"] = stiffness
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
        mode = ArmModeCommandMode(d.pop("mode"))

        def _parse_acc_ratio(data: object) -> float | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(float | None | Unset, data)

        acc_ratio = _parse_acc_ratio(d.pop("acc_ratio", UNSET))

        def _parse_adjustment_limit_mm(data: object) -> float | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(float | None | Unset, data)

        adjustment_limit_mm = _parse_adjustment_limit_mm(
            d.pop("adjustment_limit_mm", UNSET)
        )

        def _parse_anchor_command_pose(data: object) -> bool | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(bool | None | Unset, data)

        anchor_command_pose = _parse_anchor_command_pose(
            d.pop("anchor_command_pose", UNSET)
        )

        def _parse_control_params(data: object) -> list[float] | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            try:
                if not isinstance(data, list):
                    raise TypeError()
                control_params_type_0 = cast(list[float], data)

                return control_params_type_0
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            return cast(list[float] | None | Unset, data)

        control_params = _parse_control_params(d.pop("control_params", UNSET))

        def _parse_damping(data: object) -> list[float] | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            try:
                if not isinstance(data, list):
                    raise TypeError()
                damping_type_0 = cast(list[float], data)

                return damping_type_0
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            return cast(list[float] | None | Unset, data)

        damping = _parse_damping(d.pop("damping", UNSET))

        def _parse_eef_rotation(data: object) -> list[float] | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            try:
                if not isinstance(data, list):
                    raise TypeError()
                eef_rotation_type_0 = cast(list[float], data)

                return eef_rotation_type_0
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            return cast(list[float] | None | Unset, data)

        eef_rotation = _parse_eef_rotation(d.pop("eef_rotation", UNSET))

        def _parse_eef_rotation_type(data: object) -> int | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(int | None | Unset, data)

        eef_rotation_type = _parse_eef_rotation_type(d.pop("eef_rotation_type", UNSET))

        def _parse_force_direction(data: object) -> list[float] | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            try:
                if not isinstance(data, list):
                    raise TypeError()
                force_direction_type_0 = cast(list[float], data)

                return force_direction_type_0
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            return cast(list[float] | None | Unset, data)

        force_direction = _parse_force_direction(d.pop("force_direction", UNSET))

        def _parse_force_type(data: object) -> int | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(int | None | Unset, data)

        force_type = _parse_force_type(d.pop("force_type", UNSET))

        def _parse_holder(data: object) -> None | str | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(None | str | Unset, data)

        holder = _parse_holder(d.pop("holder", UNSET))

        def _parse_stiffness(data: object) -> list[float] | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            try:
                if not isinstance(data, list):
                    raise TypeError()
                stiffness_type_0 = cast(list[float], data)

                return stiffness_type_0
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            return cast(list[float] | None | Unset, data)

        stiffness = _parse_stiffness(d.pop("stiffness", UNSET))

        def _parse_target_force(data: object) -> float | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(float | None | Unset, data)

        target_force = _parse_target_force(d.pop("target_force", UNSET))

        def _parse_vel_ratio(data: object) -> float | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(float | None | Unset, data)

        vel_ratio = _parse_vel_ratio(d.pop("vel_ratio", UNSET))

        def _parse_zero_wrist_roll(data: object) -> bool | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(bool | None | Unset, data)

        zero_wrist_roll = _parse_zero_wrist_roll(d.pop("zero_wrist_roll", UNSET))

        arm_mode_command = cls(
            mode=mode,
            acc_ratio=acc_ratio,
            adjustment_limit_mm=adjustment_limit_mm,
            anchor_command_pose=anchor_command_pose,
            control_params=control_params,
            damping=damping,
            eef_rotation=eef_rotation,
            eef_rotation_type=eef_rotation_type,
            force_direction=force_direction,
            force_type=force_type,
            holder=holder,
            stiffness=stiffness,
            target_force=target_force,
            vel_ratio=vel_ratio,
            zero_wrist_roll=zero_wrist_roll,
        )

        arm_mode_command.additional_properties = d
        return arm_mode_command

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
