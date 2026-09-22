from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.vendor_ros_command_type_6_command import VendorRosCommandType6Command

T = TypeVar("T", bound="VendorRosCommandType6")


@_attrs_define
class VendorRosCommandType6:
    """Standard std_srvs/SetBool /pause request; vendor execution remains unverified.

    Attributes:
        command (VendorRosCommandType6Command):
        paused (bool): true pauses, false resumes.
    """

    command: VendorRosCommandType6Command
    paused: bool
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        command = self.command.value

        paused = self.paused

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "command": command,
                "paused": paused,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        command = VendorRosCommandType6Command(d.pop("command"))

        paused = d.pop("paused")

        vendor_ros_command_type_6 = cls(
            command=command,
            paused=paused,
        )

        vendor_ros_command_type_6.additional_properties = d
        return vendor_ros_command_type_6

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
