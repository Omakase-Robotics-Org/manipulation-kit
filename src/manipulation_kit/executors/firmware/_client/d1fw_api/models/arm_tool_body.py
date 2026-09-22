from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..types import UNSET, Unset

T = TypeVar("T", bound="ArmToolBody")


@_attrs_define
class ArmToolBody:
    """
    Attributes:
        com_mm (list[float]): Centre of mass in millimetres, XYZ.
        inertia (list[float]): Inertia tensor components in the backend's six-value order.
        mass_kg (float): Tool mass in kilograms.
        tcp_rpy_deg (list[float]): TCP rotation in degrees, roll/pitch/yaw.
        tcp_xyz_mm (list[float]): TCP translation in millimetres, XYZ.
        holder (str | Unset): The arm lease holder issuing this command. Required only while somebody holds the lease:
            with no lease held the field is ignored, and with one held a request whose `holder` does not match is refused
            with 409.
    """

    com_mm: list[float]
    inertia: list[float]
    mass_kg: float
    tcp_rpy_deg: list[float]
    tcp_xyz_mm: list[float]
    holder: str | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        com_mm = self.com_mm

        inertia = self.inertia

        mass_kg = self.mass_kg

        tcp_rpy_deg = self.tcp_rpy_deg

        tcp_xyz_mm = self.tcp_xyz_mm

        holder = self.holder

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "com_mm": com_mm,
                "inertia": inertia,
                "mass_kg": mass_kg,
                "tcp_rpy_deg": tcp_rpy_deg,
                "tcp_xyz_mm": tcp_xyz_mm,
            }
        )
        if holder is not UNSET:
            field_dict["holder"] = holder

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        com_mm = cast(list[float], d.pop("com_mm"))

        inertia = cast(list[float], d.pop("inertia"))

        mass_kg = d.pop("mass_kg")

        tcp_rpy_deg = cast(list[float], d.pop("tcp_rpy_deg"))

        tcp_xyz_mm = cast(list[float], d.pop("tcp_xyz_mm"))

        holder = d.pop("holder", UNSET)

        arm_tool_body = cls(
            com_mm=com_mm,
            inertia=inertia,
            mass_kg=mass_kg,
            tcp_rpy_deg=tcp_rpy_deg,
            tcp_xyz_mm=tcp_xyz_mm,
            holder=holder,
        )

        arm_tool_body.additional_properties = d
        return arm_tool_body

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
