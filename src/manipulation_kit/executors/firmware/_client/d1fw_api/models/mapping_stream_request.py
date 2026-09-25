from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from typing_extensions import Self

from ..types import UNSET, Unset

T = TypeVar("T", bound="MappingStreamRequest")


@_attrs_define
class MappingStreamRequest:
    """Optional client revisions suppress unchanged payloads, not status metadata.

    Attributes:
        map_modify_revision (int | None | Unset): Last retained localization-map revision; omit to receive a full frame.
        map_revision (int | None | Unset): Last retained map revision; omit to receive a full valid frame.
        scan_revision (int | None | Unset): Last retained scan revision; omit to receive a full valid scan.
        stream_id (None | str | Unset): Reader instance identity returned previously; mismatches force full payloads.
    """

    map_modify_revision: int | None | Unset = UNSET
    map_revision: int | None | Unset = UNSET
    scan_revision: int | None | Unset = UNSET
    stream_id: None | str | Unset = UNSET

    def to_dict(self) -> dict[str, Any]:
        map_modify_revision: int | None | Unset
        if isinstance(self.map_modify_revision, Unset):
            map_modify_revision = UNSET
        else:
            map_modify_revision = self.map_modify_revision

        map_revision: int | None | Unset
        if isinstance(self.map_revision, Unset):
            map_revision = UNSET
        else:
            map_revision = self.map_revision

        scan_revision: int | None | Unset
        if isinstance(self.scan_revision, Unset):
            scan_revision = UNSET
        else:
            scan_revision = self.scan_revision

        stream_id: None | str | Unset
        if isinstance(self.stream_id, Unset):
            stream_id = UNSET
        else:
            stream_id = self.stream_id

        field_dict: dict[str, Any] = {}

        field_dict.update({})
        if map_modify_revision is not UNSET:
            field_dict["map_modify_revision"] = map_modify_revision
        if map_revision is not UNSET:
            field_dict["map_revision"] = map_revision
        if scan_revision is not UNSET:
            field_dict["scan_revision"] = scan_revision
        if stream_id is not UNSET:
            field_dict["stream_id"] = stream_id

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)

        def _parse_map_modify_revision(data: object) -> int | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(int | None | Unset, data)

        map_modify_revision = _parse_map_modify_revision(
            d.pop("map_modify_revision", UNSET)
        )

        def _parse_map_revision(data: object) -> int | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(int | None | Unset, data)

        map_revision = _parse_map_revision(d.pop("map_revision", UNSET))

        def _parse_scan_revision(data: object) -> int | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(int | None | Unset, data)

        scan_revision = _parse_scan_revision(d.pop("scan_revision", UNSET))

        def _parse_stream_id(data: object) -> None | str | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(None | str | Unset, data)

        stream_id = _parse_stream_id(d.pop("stream_id", UNSET))

        mapping_stream_request = cls(
            map_modify_revision=map_modify_revision,
            map_revision=map_revision,
            scan_revision=scan_revision,
            stream_id=stream_id,
        )

        return mapping_stream_request
