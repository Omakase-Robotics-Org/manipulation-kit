from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.vendor_ros_command_type_4_command import VendorRosCommandType4Command

T = TypeVar("T", bound="VendorRosCommandType4")


@_attrs_define
class VendorRosCommandType4:
    """SleepCtrl cmd 1 wakes the chassis; cmd 0 requests sleep.

    Attributes:
        awake (bool): true wakes, false sleeps; allowed only from vendor mode 0.
        command (VendorRosCommandType4Command):
    """

    awake: bool
    command: VendorRosCommandType4Command
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        awake = self.awake

        command = self.command.value

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "awake": awake,
                "command": command,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        awake = d.pop("awake")

        command = VendorRosCommandType4Command(d.pop("command"))

        vendor_ros_command_type_4 = cls(
            awake=awake,
            command=command,
        )

        vendor_ros_command_type_4.additional_properties = d
        return vendor_ros_command_type_4

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
