from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.vendor_ros_connection import VendorRosConnection

if TYPE_CHECKING:
    from ..models.vendor_ros_topic import VendorRosTopic


T = TypeVar("T", bound="VendorRosState")


@_attrs_define
class VendorRosState:
    """Snapshot of the distinct vendor ROS transport.

    Attributes:
        connection (VendorRosConnection): Latest transport state. Connected does not imply that topics have arrived.
        last_error (None | str): Last connection/protocol error, or null.
        source (str): Always vendor_ros; this is not the daemon's ROS 2 frontend.
        stale_after_ms (int): Age at which a received message becomes stale.
        topics (list[VendorRosTopic]): One entry per subscribed topic, including ones that never produced a message.
    """

    connection: VendorRosConnection
    last_error: None | str
    source: str
    stale_after_ms: int
    topics: list[VendorRosTopic]
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        connection = self.connection.value

        last_error: None | str
        last_error = self.last_error

        source = self.source

        stale_after_ms = self.stale_after_ms

        topics = []
        for topics_item_data in self.topics:
            topics_item = topics_item_data.to_dict()
            topics.append(topics_item)

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "connection": connection,
                "last_error": last_error,
                "source": source,
                "stale_after_ms": stale_after_ms,
                "topics": topics,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.vendor_ros_topic import VendorRosTopic

        d = dict(src_dict)
        connection = VendorRosConnection(d.pop("connection"))

        def _parse_last_error(data: object) -> None | str:
            if data is None:
                return data
            return cast(None | str, data)

        last_error = _parse_last_error(d.pop("last_error"))

        source = d.pop("source")

        stale_after_ms = d.pop("stale_after_ms")

        topics = []
        _topics = d.pop("topics")
        for topics_item_data in _topics:
            topics_item = VendorRosTopic.from_dict(topics_item_data)

            topics.append(topics_item)

        vendor_ros_state = cls(
            connection=connection,
            last_error=last_error,
            source=source,
            stale_after_ms=stale_after_ms,
            topics=topics,
        )

        vendor_ros_state.additional_properties = d
        return vendor_ros_state

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
