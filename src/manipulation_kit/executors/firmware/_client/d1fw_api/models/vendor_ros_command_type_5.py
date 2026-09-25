from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.vendor_ros_command_type_5_command import VendorRosCommandType5Command

T = TypeVar("T", bound="VendorRosCommandType5")


@_attrs_define
class VendorRosCommandType5:
    """Electrical charging contact control, distinct from docking and automatic charging.

    Attributes:
        command (VendorRosCommandType5Command):
        energized (bool): true energizes the charging electrodes; false deenergizes them.
    """

    command: VendorRosCommandType5Command
    energized: bool
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        command = self.command.value

        energized = self.energized

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "command": command,
                "energized": energized,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        command = VendorRosCommandType5Command(d.pop("command"))

        energized = d.pop("energized")

        vendor_ros_command_type_5 = cls(
            command=command,
            energized=energized,
        )

        vendor_ros_command_type_5.additional_properties = d
        return vendor_ros_command_type_5

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
