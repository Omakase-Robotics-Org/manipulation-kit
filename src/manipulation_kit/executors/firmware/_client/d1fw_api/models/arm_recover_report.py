from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.arm_side import ArmSide

if TYPE_CHECKING:
    from ..models.arm_state import ArmState


T = TypeVar("T", bound="ArmRecoverReport")


@_attrs_define
class ArmRecoverReport:
    """Successful recovery to a measured-pose position hold.

    Attributes:
        elapsed_ms (int): Total elapsed milliseconds.
        initial_state (ArmState): Arm feedback and command echo.
        recovered (bool): True after the final clean-position confirmation and gap check.
        side (ArmSide): Selects one of the two physical arms.
        state (ArmState): Arm feedback and command echo.
    """

    elapsed_ms: int
    initial_state: ArmState
    recovered: bool
    side: ArmSide
    state: ArmState
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        elapsed_ms = self.elapsed_ms

        initial_state = self.initial_state.to_dict()

        recovered = self.recovered

        side = self.side.value

        state = self.state.to_dict()

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "elapsed_ms": elapsed_ms,
                "initial_state": initial_state,
                "recovered": recovered,
                "side": side,
                "state": state,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.arm_state import ArmState

        d = dict(src_dict)
        elapsed_ms = d.pop("elapsed_ms")

        initial_state = ArmState.from_dict(d.pop("initial_state"))

        recovered = d.pop("recovered")

        side = ArmSide(d.pop("side"))

        state = ArmState.from_dict(d.pop("state"))

        arm_recover_report = cls(
            elapsed_ms=elapsed_ms,
            initial_state=initial_state,
            recovered=recovered,
            side=side,
            state=state,
        )

        arm_recover_report.additional_properties = d
        return arm_recover_report

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
