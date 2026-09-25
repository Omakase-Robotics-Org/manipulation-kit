from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.vendor_ros_command_type_3_command import VendorRosCommandType3Command

T = TypeVar("T", bound="VendorRosCommandType3")


@_attrs_define
class VendorRosCommandType3:
    """AutoNav cmd 1/0 with the vendor's explicit scene argument.

    Attributes:
        command (VendorRosCommandType3Command):
        enabled (bool): Start or stop the vendor navigation routine.
        scene (str): Existing scene. Nonempty when starting; preserved in the ROS request.
    """

    command: VendorRosCommandType3Command
    enabled: bool
    scene: str
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        command = self.command.value

        enabled = self.enabled

        scene = self.scene

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "command": command,
                "enabled": enabled,
                "scene": scene,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        command = VendorRosCommandType3Command(d.pop("command"))

        enabled = d.pop("enabled")

        scene = d.pop("scene")

        vendor_ros_command_type_3 = cls(
            command=command,
            enabled=enabled,
            scene=scene,
        )

        vendor_ros_command_type_3.additional_properties = d
        return vendor_ros_command_type_3

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
