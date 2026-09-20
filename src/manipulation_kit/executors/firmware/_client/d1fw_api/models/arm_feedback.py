from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

if TYPE_CHECKING:
    from ..models.arm_state import ArmState


T = TypeVar("T", bound="ArmFeedback")


@_attrs_define
class ArmFeedback:
    """Both arms' narrowed feedback from ONE controller feedback frame.

    The arm controller reports both sides in a single datagram at its native
    rate (1 kHz on hardware, 100 Hz against the simulator), so this is the
    unit an observer receives: one frame, two sides. It exists as a core type
    so a consumer that only wants to WATCH the arm — the telemetry sampler —
    can do so through the core abstraction without depending on the backend
    crate's wider `ArmSnapshot`, which additionally carries wire-level fields
    (impedance type, commanded pose, configuration-parameter serials) that
    only the backend's own command sequencing needs.

        Attributes:
            a (ArmState): Arm feedback and command echo.
            b (ArmState): Arm feedback and command echo.
    """

    a: ArmState
    b: ArmState
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        a = self.a.to_dict()

        b = self.b.to_dict()

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "a": a,
                "b": b,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.arm_state import ArmState

        d = dict(src_dict)
        a = ArmState.from_dict(d.pop("a"))

        b = ArmState.from_dict(d.pop("b"))

        arm_feedback = cls(
            a=a,
            b=b,
        )

        arm_feedback.additional_properties = d
        return arm_feedback

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
