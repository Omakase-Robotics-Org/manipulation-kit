from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.mapping_stream_status import MappingStreamStatus

if TYPE_CHECKING:
    from ..models.mapping_grid import MappingGrid


T = TypeVar("T", bound="MappingGridReading")


@_attrs_define
class MappingGridReading:
    """One independently aged stream; payload is null when client's revision matches.

    Attributes:
        age_ms (int | None): Monotonic age of latest valid receipt.
        error (None | str): Rejection/error detail; never silently replaced with fresh old data.
        payload (MappingGrid | None):
        received_at_unix_ms (int | None): Local time of the latest valid receipt.
        revision (int): Monotonic local payload revision, retained across reconnects.
        status (MappingStreamStatus): Receipt validity; fresh receipt does not prove a changing source map.
        topic (str): Fixed vendor topic.
    """

    age_ms: int | None
    error: None | str
    payload: MappingGrid | None
    received_at_unix_ms: int | None
    revision: int
    status: MappingStreamStatus
    topic: str
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        from ..models.mapping_grid import MappingGrid

        age_ms: int | None
        age_ms = self.age_ms

        error: None | str
        error = self.error

        payload: dict[str, Any] | None
        if isinstance(self.payload, MappingGrid):
            payload = self.payload.to_dict()
        else:
            payload = self.payload

        received_at_unix_ms: int | None
        received_at_unix_ms = self.received_at_unix_ms

        revision = self.revision

        status = self.status.value

        topic = self.topic

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "age_ms": age_ms,
                "error": error,
                "payload": payload,
                "received_at_unix_ms": received_at_unix_ms,
                "revision": revision,
                "status": status,
                "topic": topic,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.mapping_grid import MappingGrid

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

        def _parse_payload(data: object) -> MappingGrid | None:
            if data is None:
                return data
            try:
                if not isinstance(data, dict):
                    raise TypeError()
                payload_type_1 = MappingGrid.from_dict(data)

                return payload_type_1
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            return cast(MappingGrid | None, data)

        payload = _parse_payload(d.pop("payload"))

        def _parse_received_at_unix_ms(data: object) -> int | None:
            if data is None:
                return data
            return cast(int | None, data)

        received_at_unix_ms = _parse_received_at_unix_ms(d.pop("received_at_unix_ms"))

        revision = d.pop("revision")

        status = MappingStreamStatus(d.pop("status"))

        topic = d.pop("topic")

        mapping_grid_reading = cls(
            age_ms=age_ms,
            error=error,
            payload=payload,
            received_at_unix_ms=received_at_unix_ms,
            revision=revision,
            status=status,
            topic=topic,
        )

        mapping_grid_reading.additional_properties = d
        return mapping_grid_reading

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
