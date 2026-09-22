from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.vendor_ros_freshness import VendorRosFreshness

T = TypeVar("T", bound="VendorRosTopic")


@_attrs_define
class VendorRosTopic:
    """One fixed topic's latest raw structured message and receipt provenance.

    Attributes:
        age_ms (int | None): Monotonic time elapsed since receipt, in milliseconds.
        error (None | str): Topic-specific subscription error; null without a recorded error.
        message (Any): Unmodified received object; values are not merged with HTTP state.
        message_type (str): Vendor-declared ROS type, not a negotiated live schema.
        received_at_unix_ms (int | None): Daemon UTC receipt time in milliseconds since Unix epoch.
        status (VendorRosFreshness): Message freshness is measured by daemon receipt, not the vendor clock.
        topic (str): Fixed ROS topic name.
    """

    age_ms: int | None
    error: None | str
    message: Any
    message_type: str
    received_at_unix_ms: int | None
    status: VendorRosFreshness
    topic: str
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        age_ms: int | None
        age_ms = self.age_ms

        error: None | str
        error = self.error

        message = self.message

        message_type = self.message_type

        received_at_unix_ms: int | None
        received_at_unix_ms = self.received_at_unix_ms

        status = self.status.value

        topic = self.topic

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "age_ms": age_ms,
                "error": error,
                "message": message,
                "message_type": message_type,
                "received_at_unix_ms": received_at_unix_ms,
                "status": status,
                "topic": topic,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)

        def _parse_age_ms(data: object) -> int | None:
            if data is None:
                return data
            return cast(int | None, data)

        age_ms = _parse_age_ms(d.pop("age_ms"))

        def _parse_error(data: object) -> None | str:
            if data is None:
                return data
            return cast(None | str, data)

        error = _parse_error(d.pop("error"))

        message = d.pop("message")

        message_type = d.pop("message_type")

        def _parse_received_at_unix_ms(data: object) -> int | None:
            if data is None:
                return data
            return cast(int | None, data)

        received_at_unix_ms = _parse_received_at_unix_ms(d.pop("received_at_unix_ms"))

        status = VendorRosFreshness(d.pop("status"))

        topic = d.pop("topic")

        vendor_ros_topic = cls(
            age_ms=age_ms,
            error=error,
            message=message,
            message_type=message_type,
            received_at_unix_ms=received_at_unix_ms,
            status=status,
            topic=topic,
        )

        vendor_ros_topic.additional_properties = d
        return vendor_ros_topic

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
