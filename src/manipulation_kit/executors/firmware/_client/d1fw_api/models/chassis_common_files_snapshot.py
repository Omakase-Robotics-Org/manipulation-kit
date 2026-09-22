from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

T = TypeVar("T", bound="ChassisCommonFilesSnapshot")


@_attrs_define
class ChassisCommonFilesSnapshot:
    """Fresh manufacturer regular-file listing, preserving unknown row metadata.

    Attributes:
        entries (list[Any]): Original rows; creation metadata is not a modification time or content revision.
        file_type (str): Explicit selected type.
        received_at_unix_ms (int): UTC Unix milliseconds at receipt.
        revision (str): Opaque digest of type and complete ordered listing.
        source (str): Fixed manufacturer list endpoint.
        write_block (None | str): Invalid/duplicate identities block mutation; unknown metadata is retained.
    """

    entries: list[Any]
    file_type: str
    received_at_unix_ms: int
    revision: str
    source: str
    write_block: None | str
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        entries = self.entries

        file_type = self.file_type

        received_at_unix_ms = self.received_at_unix_ms

        revision = self.revision

        source = self.source

        write_block: None | str
        write_block = self.write_block

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "entries": entries,
                "file_type": file_type,
                "received_at_unix_ms": received_at_unix_ms,
                "revision": revision,
                "source": source,
                "write_block": write_block,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        entries = cast(list[Any], d.pop("entries"))

        file_type = d.pop("file_type")

        received_at_unix_ms = d.pop("received_at_unix_ms")

        revision = d.pop("revision")

        source = d.pop("source")

        def _parse_write_block(data: object) -> None | str:
            if data is None:
                return data
            return cast(None | str, data)

        write_block = _parse_write_block(d.pop("write_block"))

        chassis_common_files_snapshot = cls(
            entries=entries,
            file_type=file_type,
            received_at_unix_ms=received_at_unix_ms,
            revision=revision,
            source=source,
            write_block=write_block,
        )

        chassis_common_files_snapshot.additional_properties = d
        return chassis_common_files_snapshot

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
