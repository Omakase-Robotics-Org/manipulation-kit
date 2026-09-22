from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

T = TypeVar("T", bound="DumpFile")


@_attrs_define
class DumpFile:
    """One file inside a dump.

    Attributes:
        bytes_ (int): Size in bytes.
        name (str): The file's name inside the dump directory, e.g.
            `statelog.arm.a.jsonl`. A dump directory is flat, so this is one path
            segment and never carries a `/`. It is also the only name
            [`Recorder::read_file`] accepts for this dump.
        records (int | None): Lines in the file for a JSON Lines file, `null` for anything else.
        sha256 (str): Lowercase hex SHA-256 of the file's bytes, so a dump copied off the
            robot can be checked without trusting the copy.
    """

    bytes_: int
    name: str
    records: int | None
    sha256: str
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        bytes_ = self.bytes_

        name = self.name

        records: int | None
        records = self.records

        sha256 = self.sha256

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "bytes": bytes_,
                "name": name,
                "records": records,
                "sha256": sha256,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        bytes_ = d.pop("bytes")

        name = d.pop("name")

        def _parse_records(data: object) -> int | None:
            if data is None:
                return data
            return cast(int | None, data)

        records = _parse_records(d.pop("records"))

        sha256 = d.pop("sha256")

        dump_file = cls(
            bytes_=bytes_,
            name=name,
            records=records,
            sha256=sha256,
        )

        dump_file.additional_properties = d
        return dump_file

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
