from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

if TYPE_CHECKING:
    from ..models.joint_calibration import JointCalibration


T = TypeVar("T", bound="ArmCalibration")


@_attrs_define
class ArmCalibration:
    """One arm's calibration block from `robot.ini` (e.g. `R.A0`).

    Attributes:
        arm (str): Arm section prefix as it appears in `robot.ini` (e.g. `R.A0`).
        dof (int): Degrees of freedom (`Dof`).
        joints (list[JointCalibration]): Per-joint calibration in joint-index order.
    """

    arm: str
    dof: int
    joints: list[JointCalibration]
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        arm = self.arm

        dof = self.dof

        joints = []
        for joints_item_data in self.joints:
            joints_item = joints_item_data.to_dict()
            joints.append(joints_item)

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "arm": arm,
                "dof": dof,
                "joints": joints,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.joint_calibration import JointCalibration

        d = dict(src_dict)
        arm = d.pop("arm")

        dof = d.pop("dof")

        joints = []
        _joints = d.pop("joints")
        for joints_item_data in _joints:
            joints_item = JointCalibration.from_dict(joints_item_data)

            joints.append(joints_item)

        arm_calibration = cls(
            arm=arm,
            dof=dof,
            joints=joints,
        )

        arm_calibration.additional_properties = d
        return arm_calibration

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
