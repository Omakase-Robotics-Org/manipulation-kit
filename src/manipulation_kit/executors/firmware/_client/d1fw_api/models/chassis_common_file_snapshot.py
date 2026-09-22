from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.chassis_common_file_read_status import ChassisCommonFileReadStatus

if TYPE_CHECKING:
    from ..models.chassis_common_file_download_evidence import (
        ChassisCommonFileDownloadEvidence,
    )


T = TypeVar("T", bound="ChassisCommonFileSnapshot")


@_attrs_define
class ChassisCommonFileSnapshot:
    """Raw download reading. Consumers must download bytes or preview inert text only.

    Attributes:
        byte_count (int | None): Observed content length; null for unknown readings.
        content_base64 (None | str): Canonical base64 of observed bytes; null when evidence is ambiguous.
        download (ChassisCommonFileDownloadEvidence): Header and byte-count evidence from the exact download request.
        entry (Any): Original exact list row, including unknown metadata.
        file_name (str): Exact selected basename.
        file_type (str): Explicit selected type.
        message (str): Evidence/uncertainty description; never runtime application evidence.
        received_at_unix_ms (int): UTC Unix milliseconds when reading completed.
        revision (None | str): Opaque selector+content revision; null for unknown readings.
        sha256 (None | str): SHA256 of observed content bytes, independent of selector.
        source (str): Fixed manufacturer source endpoint.
        status (ChassisCommonFileReadStatus): Marked download evidence versus an ambiguous response.
    """

    byte_count: int | None
    content_base64: None | str
    download: ChassisCommonFileDownloadEvidence
    entry: Any
    file_name: str
    file_type: str
    message: str
    received_at_unix_ms: int
    revision: None | str
    sha256: None | str
    source: str
    status: ChassisCommonFileReadStatus
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        byte_count: int | None
        byte_count = self.byte_count

        content_base64: None | str
        content_base64 = self.content_base64

        download = self.download.to_dict()

        entry = self.entry

        file_name = self.file_name

        file_type = self.file_type

        message = self.message

        received_at_unix_ms = self.received_at_unix_ms

        revision: None | str
        revision = self.revision

        sha256: None | str
        sha256 = self.sha256

        source = self.source

        status = self.status.value

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "byte_count": byte_count,
                "content_base64": content_base64,
                "download": download,
                "entry": entry,
                "file_name": file_name,
                "file_type": file_type,
                "message": message,
                "received_at_unix_ms": received_at_unix_ms,
                "revision": revision,
                "sha256": sha256,
                "source": source,
                "status": status,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.chassis_common_file_download_evidence import (
            ChassisCommonFileDownloadEvidence,
        )

        d = dict(src_dict)

        def _parse_byte_count(data: object) -> int | None:
            if data is None:
                return data
            return cast(int | None, data)

        byte_count = _parse_byte_count(d.pop("byte_count"))

        def _parse_content_base64(data: object) -> None | str:
            if data is None:
                return data
            return cast(None | str, data)

        content_base64 = _parse_content_base64(d.pop("content_base64"))

        download = ChassisCommonFileDownloadEvidence.from_dict(d.pop("download"))

        entry = d.pop("entry")

        file_name = d.pop("file_name")

        file_type = d.pop("file_type")

        message = d.pop("message")

        received_at_unix_ms = d.pop("received_at_unix_ms")

        def _parse_revision(data: object) -> None | str:
            if data is None:
                return data
            return cast(None | str, data)

        revision = _parse_revision(d.pop("revision"))

        def _parse_sha256(data: object) -> None | str:
            if data is None:
                return data
            return cast(None | str, data)

        sha256 = _parse_sha256(d.pop("sha256"))

        source = d.pop("source")

        status = ChassisCommonFileReadStatus(d.pop("status"))

        chassis_common_file_snapshot = cls(
            byte_count=byte_count,
            content_base64=content_base64,
            download=download,
            entry=entry,
            file_name=file_name,
            file_type=file_type,
            message=message,
            received_at_unix_ms=received_at_unix_ms,
            revision=revision,
            sha256=sha256,
            source=source,
            status=status,
        )

        chassis_common_file_snapshot.additional_properties = d
        return chassis_common_file_snapshot

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
