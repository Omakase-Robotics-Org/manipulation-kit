from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.advisory_code import AdvisoryCode
from ..models.advisory_severity import AdvisorySeverity
from ..types import UNSET, Unset

if TYPE_CHECKING:
    from ..models.suggested_action import SuggestedAction


T = TypeVar("T", bound="Advisory")


@_attrs_define
class Advisory:
    """One suggestion about one device's current state.

    Additive everywhere it appears: absent when the state earns none.

        Attributes:
            code (AdvisoryCode): A stable machine word for one advisory, independent of its wording.

                A consumer branches on this, never on [`Advisory::message`] — the same
                rule [`crate::ErrorKind`] states for failures, for the same reason.
            message (str): One sentence for a human, naming the state and the next step.
            severity (AdvisorySeverity): How much of an operator's attention one advisory asks for.

                An enum rather than a bare string so a later advisory can be published at
                a higher severity without any consumer changing the shape it parses.
                Today exactly one severity is produced: both the arm's idle state and the
                lift's uncommissioned origin are states the machine is legitimately in,
                so they are information and nothing more.
            suggested_action (None | SuggestedAction | Unset):
    """

    code: AdvisoryCode
    message: str
    severity: AdvisorySeverity
    suggested_action: None | SuggestedAction | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        from ..models.suggested_action import SuggestedAction

        code = self.code.value

        message = self.message

        severity = self.severity.value

        suggested_action: dict[str, Any] | None | Unset
        if isinstance(self.suggested_action, Unset):
            suggested_action = UNSET
        elif isinstance(self.suggested_action, SuggestedAction):
            suggested_action = self.suggested_action.to_dict()
        else:
            suggested_action = self.suggested_action

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "code": code,
                "message": message,
                "severity": severity,
            }
        )
        if suggested_action is not UNSET:
            field_dict["suggested_action"] = suggested_action

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.suggested_action import SuggestedAction

        d = dict(src_dict)
        code = AdvisoryCode(d.pop("code"))

        message = d.pop("message")

        severity = AdvisorySeverity(d.pop("severity"))

        def _parse_suggested_action(data: object) -> None | SuggestedAction | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            try:
                if not isinstance(data, dict):
                    raise TypeError()
                suggested_action_type_1 = SuggestedAction.from_dict(data)

                return suggested_action_type_1
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            return cast(None | SuggestedAction | Unset, data)

        suggested_action = _parse_suggested_action(d.pop("suggested_action", UNSET))

        advisory = cls(
            code=code,
            message=message,
            severity=severity,
            suggested_action=suggested_action,
        )

        advisory.additional_properties = d
        return advisory

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
