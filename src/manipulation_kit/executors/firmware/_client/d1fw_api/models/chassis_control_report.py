from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.control_verb import ControlVerb

if TYPE_CHECKING:
    from ..models.chassis_control_state import ChassisControlState
    from ..models.control_step import ControlStep


T = TypeVar("T", bound="ChassisControlReport")


@_attrs_define
class ChassisControlReport:
    """What one control verb did, and the state the base was in afterwards.

    The steps are here because a control verb is not always one vendor call:
    entering remote control from navigation mode closes navigation, waits for
    the base to actually leave that mode, and only then enters remote control.
    An operator interface that shows only the end state cannot tell that apart
    from a single call, and an operator who has just lost a navigation goal
    deserves to be told that this is what took it.

        Attributes:
            control (ChassisControlState): Everything the daemon knows about WHICH CONTROLLER may drive the mobile
                base: the mode it is in, what that mode permits, and which control verbs
                are legal from it.

                The mobile base has exactly one control mode at a time and every motion
                path is gated on it, but the vendor publishes that only as a bare
                `workMode` digit and enforces it by IGNORING commands: outside its
                remote-control mode the base answers a jog with HTTP 200 and does not
                move. This object is the daemon's answer to "what state is the base in,
                what can I do from here, and why not" in one read, so that an operator
                interface never has to infer it from a digit or from a command that
                silently did nothing.

                The five vendor modes are the vendor's own, taken from its web
                application's own translation table: `0` navigation closed, `1`
                navigation, `2` automatic charging, `3` mapping, `4` remote control.
            steps (list[ControlStep]): What the daemon sent to the base, in the order it sent it.
            verb (ControlVerb): One control verb of the mobile base's mode machine.

                These are the verbs that change WHICH controller may drive the base, as
                opposed to the verbs that drive it. They are named here rather than only
                as routes so that [`ChassisControlState::transitions`] can say which of
                them are legal from the state the base is in right now.
    """

    control: ChassisControlState
    steps: list[ControlStep]
    verb: ControlVerb
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        control = self.control.to_dict()

        steps = []
        for steps_item_data in self.steps:
            steps_item = steps_item_data.to_dict()
            steps.append(steps_item)

        verb = self.verb.value

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "control": control,
                "steps": steps,
                "verb": verb,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.chassis_control_state import ChassisControlState
        from ..models.control_step import ControlStep

        d = dict(src_dict)
        control = ChassisControlState.from_dict(d.pop("control"))

        steps = []
        _steps = d.pop("steps")
        for steps_item_data in _steps:
            steps_item = ControlStep.from_dict(steps_item_data)

            steps.append(steps_item)

        verb = ControlVerb(d.pop("verb"))

        chassis_control_report = cls(
            control=control,
            steps=steps,
            verb=verb,
        )

        chassis_control_report.additional_properties = d
        return chassis_control_report

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
