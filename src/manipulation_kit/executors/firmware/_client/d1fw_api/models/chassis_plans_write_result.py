from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.chassis_plans_write_status import ChassisPlansWriteStatus

if TYPE_CHECKING:
    from ..models.chassis_plans_snapshot import ChassisPlansSnapshot


T = TypeVar("T", bound="ChassisPlansWriteResult")


@_attrs_define
class ChassisPlansWriteResult:
    """No automatic retry or rollback follows any attempted write.

    Attributes:
        expected_plans (Any): Complete collection expected after manufacturer positional renumbering.
        message (str): Readback or uncertainty detail; reconcile before any further write.
        snapshot (ChassisPlansSnapshot | None):
        status (ChassisPlansWriteStatus): Exact storage comparison result; never physical execution evidence.
    """

    expected_plans: Any
    message: str
    snapshot: ChassisPlansSnapshot | None
    status: ChassisPlansWriteStatus
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        from ..models.chassis_plans_snapshot import (
            ChassisPlansSnapshot,
        )

        expected_plans = self.expected_plans

        message = self.message

        snapshot: dict[str, Any] | None
        if isinstance(self.snapshot, ChassisPlansSnapshot):
            snapshot = self.snapshot.to_dict()
        else:
            snapshot = self.snapshot

        status = self.status.value

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "expected_plans": expected_plans,
                "message": message,
                "snapshot": snapshot,
                "status": status,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.chassis_plans_snapshot import (
            ChassisPlansSnapshot,
        )

        d = dict(src_dict)
        expected_plans = d.pop("expected_plans")

        message = d.pop("message")

        def _parse_snapshot(data: object) -> ChassisPlansSnapshot | None:
            if data is None:
                return data
            try:
                if not isinstance(data, dict):
                    raise TypeError()
                snapshot_type_1 = ChassisPlansSnapshot.from_dict(data)

                return snapshot_type_1
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            return cast(ChassisPlansSnapshot | None, data)

        snapshot = _parse_snapshot(d.pop("snapshot"))

        status = ChassisPlansWriteStatus(d.pop("status"))

        chassis_plans_write_result = cls(
            expected_plans=expected_plans,
            message=message,
            snapshot=snapshot,
            status=status,
        )

        chassis_plans_write_result.additional_properties = d
        return chassis_plans_write_result

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
