from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.arm_side import ArmSide
from ..types import UNSET, Unset

T = TypeVar("T", bound="GripperFaultReport")


@_attrs_define
class GripperFaultReport:
    """What one gripper motor reports after a clear-fault attempt.

    A DM motor that has latched a fault swallows position commands until it is
    cleared, and until 2026-09-22 nothing in this daemon could ask it to let
    go: every stroke after the fault was accepted, ended immediately as
    `fault`, and the only recovery was a power cycle of the gripper. This is
    what `POST /v1/gripper/{side}/clear_fault` answers with.

        Attributes:
            cleared (bool): Whether the motor is STILL reporting a fault.

                `false` is the success case. `true` means the clear did not take:
                `coil_over_temperature` cannot be cleared in software until the coil
                has cooled, and a motor that re-faults instantly has something else
                wrong with it.
            side (ArmSide): Selects one of the two physical arms.
            status (str): The motor's decoded status AFTER the clear: `disabled`, `enabled`, one
                of the fault names, or `unknown`.
            status_nibble (int): The raw status nibble the name was decoded from, `0x0..=0xf`.
            fault_code (None | str | Unset): The fault still standing, when `cleared` is `false`.
    """

    cleared: bool
    side: ArmSide
    status: str
    status_nibble: int
    fault_code: None | str | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        cleared = self.cleared

        side = self.side.value

        status = self.status

        status_nibble = self.status_nibble

        fault_code: None | str | Unset
        if isinstance(self.fault_code, Unset):
            fault_code = UNSET
        else:
            fault_code = self.fault_code

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "cleared": cleared,
                "side": side,
                "status": status,
                "status_nibble": status_nibble,
            }
        )
        if fault_code is not UNSET:
            field_dict["fault_code"] = fault_code

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        cleared = d.pop("cleared")

        side = ArmSide(d.pop("side"))

        status = d.pop("status")

        status_nibble = d.pop("status_nibble")

        def _parse_fault_code(data: object) -> None | str | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(None | str | Unset, data)

        fault_code = _parse_fault_code(d.pop("fault_code", UNSET))

        gripper_fault_report = cls(
            cleared=cleared,
            side=side,
            status=status,
            status_nibble=status_nibble,
            fault_code=fault_code,
        )

        gripper_fault_report.additional_properties = d
        return gripper_fault_report

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
