from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..types import UNSET, Unset

T = TypeVar("T", bound="JointCalibration")


@_attrs_define
class JointCalibration:
    """Per-joint calibration read from the board's `robot.ini` (the `.BASIC`
    section of each joint).

        Attributes:
            enc_offset (int): Encoder offset (`EncOffset`).
            enc_res (int): Encoder resolution (`EncRes`, counts per revolution).
            joint (int): Joint index (0-based; `L0`..`L6`).
            limit_neg_deg (float): Negative joint limit in degrees (`LimitNeg`).
            limit_pos_deg (float): Positive joint limit in degrees (`LimitPos`).
            torque_max (float): Maximum torque (`TorqueMax`, from the joint's `.BASIC` section — not the
                separate `.MOTOR` section's rated torque).
            acc_max (float | None | Unset): Maximum acceleration (`AccMax`) if present.
            vel_max (float | None | Unset): Maximum velocity (`VelMax`) if present.
    """

    enc_offset: int
    enc_res: int
    joint: int
    limit_neg_deg: float
    limit_pos_deg: float
    torque_max: float
    acc_max: float | None | Unset = UNSET
    vel_max: float | None | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        enc_offset = self.enc_offset

        enc_res = self.enc_res

        joint = self.joint

        limit_neg_deg = self.limit_neg_deg

        limit_pos_deg = self.limit_pos_deg

        torque_max = self.torque_max

        acc_max: float | None | Unset
        if isinstance(self.acc_max, Unset):
            acc_max = UNSET
        else:
            acc_max = self.acc_max

        vel_max: float | None | Unset
        if isinstance(self.vel_max, Unset):
            vel_max = UNSET
        else:
            vel_max = self.vel_max

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "enc_offset": enc_offset,
                "enc_res": enc_res,
                "joint": joint,
                "limit_neg_deg": limit_neg_deg,
                "limit_pos_deg": limit_pos_deg,
                "torque_max": torque_max,
            }
        )
        if acc_max is not UNSET:
            field_dict["acc_max"] = acc_max
        if vel_max is not UNSET:
            field_dict["vel_max"] = vel_max

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        enc_offset = d.pop("enc_offset")

        enc_res = d.pop("enc_res")

        joint = d.pop("joint")

        limit_neg_deg = d.pop("limit_neg_deg")

        limit_pos_deg = d.pop("limit_pos_deg")

        torque_max = d.pop("torque_max")

        def _parse_acc_max(data: object) -> float | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(float | None | Unset, data)

        acc_max = _parse_acc_max(d.pop("acc_max", UNSET))

        def _parse_vel_max(data: object) -> float | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(float | None | Unset, data)

        vel_max = _parse_vel_max(d.pop("vel_max", UNSET))

        joint_calibration = cls(
            enc_offset=enc_offset,
            enc_res=enc_res,
            joint=joint,
            limit_neg_deg=limit_neg_deg,
            limit_pos_deg=limit_pos_deg,
            torque_max=torque_max,
            acc_max=acc_max,
            vel_max=vel_max,
        )

        joint_calibration.additional_properties = d
        return joint_calibration

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
