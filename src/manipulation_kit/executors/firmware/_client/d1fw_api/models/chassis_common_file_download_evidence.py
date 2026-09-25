from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

T = TypeVar("T", bound="ChassisCommonFileDownloadEvidence")


@_attrs_define
class ChassisCommonFileDownloadEvidence:
    """Header and byte-count evidence from the exact download request.

    Attributes:
        content_disposition (None | str): Original Content-Disposition, if present and textual.
        content_type (None | str): Original Content-Type, if present and textual.
        declared_length (int | None): Original valid Content-Length, absent for an unmarked/chunked response.
        http_status (int): Original upstream HTTP status.
        received_bytes (int): Actual response bytes received under the product bound.
    """

    content_disposition: None | str
    content_type: None | str
    declared_length: int | None
    http_status: int
    received_bytes: int
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        content_disposition: None | str
        content_disposition = self.content_disposition

        content_type: None | str
        content_type = self.content_type

        declared_length: int | None
        declared_length = self.declared_length

        http_status = self.http_status

        received_bytes = self.received_bytes

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "content_disposition": content_disposition,
                "content_type": content_type,
                "declared_length": declared_length,
                "http_status": http_status,
                "received_bytes": received_bytes,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)

        def _parse_content_disposition(data: object) -> None | str:
            if data is None:
                return data
            return cast(None | str, data)

        content_disposition = _parse_content_disposition(d.pop("content_disposition"))

        def _parse_content_type(data: object) -> None | str:
            if data is None:
                return data
            return cast(None | str, data)

        content_type = _parse_content_type(d.pop("content_type"))

        def _parse_declared_length(data: object) -> int | None:
            if data is None:
                return data
            return cast(int | None, data)

        declared_length = _parse_declared_length(d.pop("declared_length"))

        http_status = d.pop("http_status")

        received_bytes = d.pop("received_bytes")

        chassis_common_file_download_evidence = cls(
            content_disposition=content_disposition,
            content_type=content_type,
            declared_length=declared_length,
            http_status=http_status,
            received_bytes=received_bytes,
        )

        chassis_common_file_download_evidence.additional_properties = d
        return chassis_common_file_download_evidence

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
