from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

T = TypeVar("T", bound="VendorRosCapability")


@_attrs_define
class VendorRosCapability:
    """An operation's implementation status, not hardware support or completion.

    Attributes:
        command (None | str): Supported command family, or null for unsupported/read-only features.
        id (str): Stable audited feature ID R01 through R22.
        reason (str): Implementation and evidence limits; never proof of physical success.
        supported (bool): Whether this daemon implements the bounded transport route.
        topic (None | str): Fixed read-only topic, or null for command features.
    """

    command: None | str
    id: str
    reason: str
    supported: bool
    topic: None | str
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        command: None | str
        command = self.command

        id = self.id

        reason = self.reason

        supported = self.supported

        topic: None | str
        topic = self.topic

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "command": command,
                "id": id,
                "reason": reason,
                "supported": supported,
                "topic": topic,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)

        def _parse_command(data: object) -> None | str:
            if data is None:
                return data
            return cast(None | str, data)

        command = _parse_command(d.pop("command"))

        id = d.pop("id")

        reason = d.pop("reason")

        supported = d.pop("supported")

        def _parse_topic(data: object) -> None | str:
            if data is None:
                return data
            return cast(None | str, data)

        topic = _parse_topic(d.pop("topic"))

        vendor_ros_capability = cls(
            command=command,
            id=id,
            reason=reason,
            supported=supported,
            topic=topic,
        )

        vendor_ros_capability.additional_properties = d
        return vendor_ros_capability

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
