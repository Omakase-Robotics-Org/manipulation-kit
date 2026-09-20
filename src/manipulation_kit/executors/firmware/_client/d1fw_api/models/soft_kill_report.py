from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

if TYPE_CHECKING:
    from ..models.kill_snapshot import KillSnapshot
    from ..models.soft_kill_action import SoftKillAction


T = TypeVar("T", bound="SoftKillReport")


@_attrs_define
class SoftKillReport:
    """The outcome of one whole soft kill.

    The latch — the part that is always effective, applied before any stop is
    issued — is in `latched`; `actions` is what each device's stop-shaped verb
    then did. A kill whose report contains failures is still a kill: nothing
    new is admitted for any device in `latched`.

        Attributes:
            actions (list[SoftKillAction]): Every stop action, in a stable order.
            latched (KillSnapshot): A copyable snapshot of every device's soft-kill latch.
    """

    actions: list[SoftKillAction]
    latched: KillSnapshot
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        actions = []
        for actions_item_data in self.actions:
            actions_item = actions_item_data.to_dict()
            actions.append(actions_item)

        latched = self.latched.to_dict()

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "actions": actions,
                "latched": latched,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.kill_snapshot import KillSnapshot
        from ..models.soft_kill_action import SoftKillAction

        d = dict(src_dict)
        actions = []
        _actions = d.pop("actions")
        for actions_item_data in _actions:
            actions_item = SoftKillAction.from_dict(actions_item_data)

            actions.append(actions_item)

        latched = KillSnapshot.from_dict(d.pop("latched"))

        soft_kill_report = cls(
            actions=actions,
            latched=latched,
        )

        soft_kill_report.additional_properties = d
        return soft_kill_report

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
