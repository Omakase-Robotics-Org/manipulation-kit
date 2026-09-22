from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.vendor_ros_command_type_2_command import VendorRosCommandType2Command

T = TypeVar("T", bound="VendorRosCommandType2")


@_attrs_define
class VendorRosCommandType2:
    """AutoCharge cmd 1/0, independently of HTTP navigation stop.

    Attributes:
        command (VendorRosCommandType2Command):
        enabled (bool): true starts; false requests end. Either can cause motion.
    """

    command: VendorRosCommandType2Command
    enabled: bool
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        command = self.command.value

        enabled = self.enabled

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "command": command,
                "enabled": enabled,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        command = VendorRosCommandType2Command(d.pop("command"))

        enabled = d.pop("enabled")

        vendor_ros_command_type_2 = cls(
            command=command,
            enabled=enabled,
        )

        vendor_ros_command_type_2.additional_properties = d
        return vendor_ros_command_type_2

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
