from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.chassis_common_file_write_status import ChassisCommonFileWriteStatus

if TYPE_CHECKING:
    from ..models.chassis_common_file_snapshot import ChassisCommonFileSnapshot
    from ..models.chassis_common_files_snapshot import ChassisCommonFilesSnapshot


T = TypeVar("T", bound="ChassisCommonFileWriteResult")


@_attrs_define
class ChassisCommonFileWriteResult:
    """One send, no automatic retry or rollback; explicitly reread and reconcile uncertainty.

    Attributes:
        expected_sha256 (None | str): SHA256 of requested upload/replacement bytes; null for delete.
        listing (ChassisCommonFilesSnapshot | None):
        message (str): Exact verification scope or uncertainty, with observed errors.
        snapshot (ChassisCommonFileSnapshot | None):
        status (ChassisCommonFileWriteStatus): Mutation verification concerns observed storage only.
    """

    expected_sha256: None | str
    listing: ChassisCommonFilesSnapshot | None
    message: str
    snapshot: ChassisCommonFileSnapshot | None
    status: ChassisCommonFileWriteStatus
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        from ..models.chassis_common_file_snapshot import (
            ChassisCommonFileSnapshot,
        )
        from ..models.chassis_common_files_snapshot import (
            ChassisCommonFilesSnapshot,
        )

        expected_sha256: None | str
        expected_sha256 = self.expected_sha256

        listing: dict[str, Any] | None
        if isinstance(self.listing, ChassisCommonFilesSnapshot):
            listing = self.listing.to_dict()
        else:
            listing = self.listing

        message = self.message

        snapshot: dict[str, Any] | None
        if isinstance(self.snapshot, ChassisCommonFileSnapshot):
            snapshot = self.snapshot.to_dict()
        else:
            snapshot = self.snapshot

        status = self.status.value

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "expected_sha256": expected_sha256,
                "listing": listing,
                "message": message,
                "snapshot": snapshot,
                "status": status,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.chassis_common_file_snapshot import (
            ChassisCommonFileSnapshot,
        )
        from ..models.chassis_common_files_snapshot import (
            ChassisCommonFilesSnapshot,
        )

        d = dict(src_dict)

        def _parse_expected_sha256(data: object) -> None | str:
            if data is None:
                return data
            return cast(None | str, data)

        expected_sha256 = _parse_expected_sha256(d.pop("expected_sha256"))

        def _parse_listing(data: object) -> ChassisCommonFilesSnapshot | None:
            if data is None:
                return data
            try:
                if not isinstance(data, dict):
                    raise TypeError()
                listing_type_1 = ChassisCommonFilesSnapshot.from_dict(data)

                return listing_type_1
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            return cast(ChassisCommonFilesSnapshot | None, data)

        listing = _parse_listing(d.pop("listing"))

        message = d.pop("message")

        def _parse_snapshot(data: object) -> ChassisCommonFileSnapshot | None:
            if data is None:
                return data
            try:
                if not isinstance(data, dict):
                    raise TypeError()
                snapshot_type_1 = ChassisCommonFileSnapshot.from_dict(data)

                return snapshot_type_1
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            return cast(ChassisCommonFileSnapshot | None, data)

        snapshot = _parse_snapshot(d.pop("snapshot"))

        status = ChassisCommonFileWriteStatus(d.pop("status"))

        chassis_common_file_write_result = cls(
            expected_sha256=expected_sha256,
            listing=listing,
            message=message,
            snapshot=snapshot,
            status=status,
        )

        chassis_common_file_write_result.additional_properties = d
        return chassis_common_file_write_result

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
