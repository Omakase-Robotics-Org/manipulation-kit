from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.stroke_kind import StrokeKind
from ..types import UNSET, Unset

T = TypeVar("T", bound="GripperReport")


@_attrs_define
class GripperReport:
    """A gripper stroke report.

    Attributes:
        grip_preload_rad (float): Standing preload the hold keeps, in motor radians; zero when nothing
            is held.
        holding (bool): Whether the report indicates an object is held.
        jaw_rad (float): Measured jaw position in motor radians.
        kind (StrokeKind): The outcome category of a gripper stroke.
        torque_nm (float): Measured motor torque in newton-metres.
        coil_c (int | None | Unset): Coil temperature in °C from a live reading, when one was taken.
        live (bool | Unset): Whether `jaw_rad` and `torque_nm` are a live reading of the idle motor
            (true) or the last stroke's telemetry (false, motor busy or silent).
        open_rad (float | None | Unset): Configured open ceiling in motor radians, when published by the backend.
    """

    grip_preload_rad: float
    holding: bool
    jaw_rad: float
    kind: StrokeKind
    torque_nm: float
    coil_c: int | None | Unset = UNSET
    live: bool | Unset = UNSET
    open_rad: float | None | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        grip_preload_rad = self.grip_preload_rad

        holding = self.holding

        jaw_rad = self.jaw_rad

        kind = self.kind.value

        torque_nm = self.torque_nm

        coil_c: int | None | Unset
        if isinstance(self.coil_c, Unset):
            coil_c = UNSET
        else:
            coil_c = self.coil_c

        live = self.live

        open_rad: float | None | Unset
        if isinstance(self.open_rad, Unset):
            open_rad = UNSET
        else:
            open_rad = self.open_rad

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "grip_preload_rad": grip_preload_rad,
                "holding": holding,
                "jaw_rad": jaw_rad,
                "kind": kind,
                "torque_nm": torque_nm,
            }
        )
        if coil_c is not UNSET:
            field_dict["coil_c"] = coil_c
        if live is not UNSET:
            field_dict["live"] = live
        if open_rad is not UNSET:
            field_dict["open_rad"] = open_rad

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        grip_preload_rad = d.pop("grip_preload_rad")

        holding = d.pop("holding")

        jaw_rad = d.pop("jaw_rad")

        kind = StrokeKind(d.pop("kind"))

        torque_nm = d.pop("torque_nm")

        def _parse_coil_c(data: object) -> int | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(int | None | Unset, data)

        coil_c = _parse_coil_c(d.pop("coil_c", UNSET))

        live = d.pop("live", UNSET)

        def _parse_open_rad(data: object) -> float | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(float | None | Unset, data)

        open_rad = _parse_open_rad(d.pop("open_rad", UNSET))

        gripper_report = cls(
            grip_preload_rad=grip_preload_rad,
            holding=holding,
            jaw_rad=jaw_rad,
            kind=kind,
            torque_nm=torque_nm,
            coil_c=coil_c,
            live=live,
            open_rad=open_rad,
        )

        gripper_report.additional_properties = d
        return gripper_report

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
