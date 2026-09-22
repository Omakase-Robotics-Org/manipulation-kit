from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

if TYPE_CHECKING:
    from ..models.vendor_ros_capability import VendorRosCapability


T = TypeVar("T", bound="VendorRosCapabilities")


@_attrs_define
class VendorRosCapabilities:
    """Inventory includes unsupported capabilities and the separate actuation opt-in.

    Attributes:
        capabilities (list[VendorRosCapability]): All 22 audited features; supported does not imply live verification.
            A row with supported=false is not implemented here; for the read-only
            topic rows R16-R22 that means the vendor rosbridge's closed topic
            allow-list does not serve the topic, so no message can ever arrive and
            the reason field says so.
        commands_enabled (bool): Device writes have a separate configuration opt-in.
        configured (bool): Explicit bridge URL is configured.
    """

    capabilities: list[VendorRosCapability]
    commands_enabled: bool
    configured: bool
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        capabilities = []
        for capabilities_item_data in self.capabilities:
            capabilities_item = capabilities_item_data.to_dict()
            capabilities.append(capabilities_item)

        commands_enabled = self.commands_enabled

        configured = self.configured

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "capabilities": capabilities,
                "commands_enabled": commands_enabled,
                "configured": configured,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.vendor_ros_capability import VendorRosCapability

        d = dict(src_dict)
        capabilities = []
        _capabilities = d.pop("capabilities")
        for capabilities_item_data in _capabilities:
            capabilities_item = VendorRosCapability.from_dict(capabilities_item_data)

            capabilities.append(capabilities_item)

        commands_enabled = d.pop("commands_enabled")

        configured = d.pop("configured")

        vendor_ros_capabilities = cls(
            capabilities=capabilities,
            commands_enabled=commands_enabled,
            configured=configured,
        )

        vendor_ros_capabilities.additional_properties = d
        return vendor_ros_capabilities

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
