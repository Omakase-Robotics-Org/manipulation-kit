from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.vendor_mapping_action import VendorMappingAction
from ..models.vendor_ros_command_type_1_command import VendorRosCommandType1Command

if TYPE_CHECKING:
    from ..models.vendor_mapping_expected import VendorMappingExpected


T = TypeVar("T", bound="VendorRosCommandType1")


@_attrs_define
class VendorRosCommandType1:
    """Mapping changes trajectories and persistent map files; never an implicit sleep command.

    Attributes:
        action (VendorMappingAction): Fixed mapping operation; numeric vendor modes are not caller-controlled.
        command (VendorRosCommandType1Command):
        expected (VendorMappingExpected): Operator-observed mode precondition, checked again immediately before sending.
        scene (str): Single scene filesystem component; continuation must match current scene.
    """

    action: VendorMappingAction
    command: VendorRosCommandType1Command
    expected: VendorMappingExpected
    scene: str
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        action = self.action.value

        command = self.command.value

        expected = self.expected.to_dict()

        scene = self.scene

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "action": action,
                "command": command,
                "expected": expected,
                "scene": scene,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.vendor_mapping_expected import (
            VendorMappingExpected,
        )

        d = dict(src_dict)
        action = VendorMappingAction(d.pop("action"))

        command = VendorRosCommandType1Command(d.pop("command"))

        expected = VendorMappingExpected.from_dict(d.pop("expected"))

        scene = d.pop("scene")

        vendor_ros_command_type_1 = cls(
            action=action,
            command=command,
            expected=expected,
            scene=scene,
        )

        vendor_ros_command_type_1.additional_properties = d
        return vendor_ros_command_type_1

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
