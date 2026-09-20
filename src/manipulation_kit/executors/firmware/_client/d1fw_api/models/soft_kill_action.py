from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.soft_kill_outcome import SoftKillOutcome
from ..types import UNSET, Unset

T = TypeVar("T", bound="SoftKillAction")


@_attrs_define
class SoftKillAction:
    """One stop action issued by a soft kill, and what became of it.

    Attributes:
        action (str): Dotted action name, such as `arm.a.emergency_stop` or
            `chassis.cmd_vel_zero`.
        elapsed_ms (int): Milliseconds from the moment the kill was raised to this action
            settling.

            Every action is issued concurrently, so these are elapsed times from
            one shared origin rather than a sum: an arm that stopped in 3 ms next
            to a chassis that timed out at 1000 ms is the join working as
            intended, not 1003 ms of kill latency.
        outcome (SoftKillOutcome): What one soft-kill stop action did.
        detail (None | str | Unset): Why, when the outcome is not `ok`. Absent on success.
    """

    action: str
    elapsed_ms: int
    outcome: SoftKillOutcome
    detail: None | str | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        action = self.action

        elapsed_ms = self.elapsed_ms

        outcome = self.outcome.value

        detail: None | str | Unset
        if isinstance(self.detail, Unset):
            detail = UNSET
        else:
            detail = self.detail

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "action": action,
                "elapsed_ms": elapsed_ms,
                "outcome": outcome,
            }
        )
        if detail is not UNSET:
            field_dict["detail"] = detail

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        action = d.pop("action")

        elapsed_ms = d.pop("elapsed_ms")

        outcome = SoftKillOutcome(d.pop("outcome"))

        def _parse_detail(data: object) -> None | str | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(None | str | Unset, data)

        detail = _parse_detail(d.pop("detail", UNSET))

        soft_kill_action = cls(
            action=action,
            elapsed_ms=elapsed_ms,
            outcome=outcome,
            detail=detail,
        )

        soft_kill_action.additional_properties = d
        return soft_kill_action

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
