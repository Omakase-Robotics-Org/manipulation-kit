from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

T = TypeVar("T", bound="ChassisVelocity")


@_attrs_define
class ChassisVelocity:
    """A chassis velocity command.

    Attributes:
        omega (float): Angular velocity in radians per second; positive is counter-clockwise.
        vx (float): Forward velocity in metres per second; negative is reverse.
    """

    omega: float
    vx: float
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        omega = self.omega

        vx = self.vx

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "omega": omega,
                "vx": vx,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        omega = d.pop("omega")

        vx = d.pop("vx")

        chassis_velocity = cls(
            omega=omega,
            vx=vx,
        )

        chassis_velocity.additional_properties = d
        return chassis_velocity

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
