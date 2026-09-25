from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

T = TypeVar("T", bound="DumpNamespace")


@_attrs_define
class DumpNamespace:
    """How far back one namespace's copied ring reaches.

    Attributes:
        newest_ts (None | str): The `ts` of the last record, or `null` when the file holds none.
        ns (str): The dotted state-log namespace, e.g. `arm.a`.
        oldest_ts (None | str): The `ts` of the first record, or `null` when the file holds none.
        records (int): Records in it, the `meta` segment headers included.
        segments (int): Segments concatenated into this namespace's file.
    """

    newest_ts: None | str
    ns: str
    oldest_ts: None | str
    records: int
    segments: int
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        newest_ts: None | str
        newest_ts = self.newest_ts

        ns = self.ns

        oldest_ts: None | str
        oldest_ts = self.oldest_ts

        records = self.records

        segments = self.segments

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "newest_ts": newest_ts,
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

        def _parse_newest_ts(data: object) -> None | str:
            if data is None:
                return data
            return cast(None | str, data)

        newest_ts = _parse_newest_ts(d.pop("newest_ts"))

        ns = d.pop("ns")

        def _parse_oldest_ts(data: object) -> None | str:
            if data is None:
                return data
            return cast(None | str, data)

        oldest_ts = _parse_oldest_ts(d.pop("oldest_ts"))

        records = d.pop("records")

        segments = d.pop("segments")

        dump_namespace = cls(
            newest_ts=newest_ts,
            ns=ns,
            oldest_ts=oldest_ts,
            records=records,
            segments=segments,
        )

        dump_namespace.additional_properties = d
        return dump_namespace

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
