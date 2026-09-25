from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.trigger_kind import TriggerKind

T = TypeVar("T", bound="DumpTrigger")


@_attrs_define
class DumpTrigger:
    """One trigger: what raised it, when, and what it saw.

    Attributes:
        detail (None | str): What the detector saw, in one sentence, or `null` when the kind says
            everything there is to say. Prose for a human; branch on `kind`.
        kind (TriggerKind): What caused a dump.

            Deliberately an enum rather than a string, and deliberately open to
            growth: the mobile base's `getRobotInfo` document carries `stopStatus` and
            `stripStatus` next to the `emergencyStatus` this daemon already watches
            (a released switch on D1 #1 reads `emergencyStatus:"0"`), and an arm whose
            controller reports a non-zero `error_code` is the other obvious candidate.
            None of them is implemented here: a kind exists once something PUBLISHES
            it, because a kind nothing can raise is a contract with no behaviour
            behind it.
        ts (str): When it was observed, UTC RFC 3339 with millisecond precision.
    """

    detail: None | str
    kind: TriggerKind
    ts: str
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        detail: None | str
        detail = self.detail

        kind = self.kind.value

        ts = self.ts

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "detail": detail,
                "kind": kind,
                "ts": ts,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)

        def _parse_detail(data: object) -> None | str:
            if data is None:
                return data
            return cast(None | str, data)

        detail = _parse_detail(d.pop("detail"))

        kind = TriggerKind(d.pop("kind"))

        ts = d.pop("ts")

        dump_trigger = cls(
            detail=detail,
            kind=kind,
            ts=ts,
        )

        dump_trigger.additional_properties = d
        return dump_trigger

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
