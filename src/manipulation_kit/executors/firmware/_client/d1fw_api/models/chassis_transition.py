from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.control_verb import ControlVerb

T = TypeVar("T", bound="ChassisTransition")


@_attrs_define
class ChassisTransition:
    """Whether one [`ControlVerb`] is legal from the state the base is in, and
    why.

    `reason` is filled in BOTH cases, because both are things a consumer has
    to show: for a refused verb it says which state refused it, and for an
    allowed verb that does more than one thing it says what the daemon will
    actually do.

        Attributes:
            allowed (bool): Whether the daemon will attempt it from the current state.
            reason (str): Why it is refused, or what an allowed verb will do.
            verb (ControlVerb): One control verb of the mobile base's mode machine.

                These are the verbs that change WHICH controller may drive the base, as
                opposed to the verbs that drive it. They are named here rather than only
                as routes so that [`ChassisControlState::transitions`] can say which of
                them are legal from the state the base is in right now.
    """

    allowed: bool
    reason: str
    verb: ControlVerb
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        allowed = self.allowed

        reason = self.reason

        verb = self.verb.value

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "allowed": allowed,
                "reason": reason,
                "verb": verb,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        allowed = d.pop("allowed")

        reason = d.pop("reason")

        verb = ControlVerb(d.pop("verb"))

        chassis_transition = cls(
            allowed=allowed,
            reason=reason,
            verb=verb,
        )

        chassis_transition.additional_properties = d
        return chassis_transition

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
