from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

T = TypeVar("T", bound="RecorderNamespace")


@_attrs_define
class RecorderNamespace:
    """One state-log namespace as `GET /v1/recorder` reports it.

    Attributes:
        bytes_ (int): Bytes those segments occupy.
        ns (str): The dotted namespace name.
        oldest_ts (None | str): The `ts` of the oldest record still in the ring, or `null`.
        records (int): Records in them, the `meta` segment headers included.
        segments (int): Segments it currently holds.
    """

    bytes_: int
    ns: str
    oldest_ts: None | str
    records: int
    segments: int
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        bytes_ = self.bytes_

        ns = self.ns

        oldest_ts: None | str
        oldest_ts = self.oldest_ts

        records = self.records

        segments = self.segments

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "bytes": bytes_,
                "ns": ns,
                "oldest_ts": oldest_ts,
                "records": records,
                "segments": segments,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        bytes_ = d.pop("bytes")

        ns = d.pop("ns")

        def _parse_oldest_ts(data: object) -> None | str:
            if data is None:
                return data
            return cast(None | str, data)

        oldest_ts = _parse_oldest_ts(d.pop("oldest_ts"))

        records = d.pop("records")

        segments = d.pop("segments")

        recorder_namespace = cls(
            bytes_=bytes_,
            ns=ns,
            oldest_ts=oldest_ts,
            records=records,
            segments=segments,
        )

        recorder_namespace.additional_properties = d
        return recorder_namespace

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
