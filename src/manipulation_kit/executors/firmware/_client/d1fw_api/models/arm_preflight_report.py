from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.arm_side import ArmSide
from ..types import UNSET, Unset

if TYPE_CHECKING:
    from ..models.advisory import Advisory
    from ..models.arm_controller_servos import ArmControllerServos
    from ..models.arm_state import ArmState


T = TypeVar("T", bound="ArmPreflightReport")


@_attrs_define
class ArmPreflightReport:
    """Read-only assessment, not a reservation or authorization for future motion.

    Attributes:
        blocking (list[str]): Actionable reasons preventing a position-mode request.
        commanded_gap_deg (float): Maximum absolute command/feedback difference, in degrees.
        feedback_advancing (bool): Whether feedback advanced during the observation period.
        initial_frame_serial (int): Frame serial before the 300 ms observation period.
        max_velocity_deg_s (float): Maximum absolute feedback velocity, in degrees per second.
        ready (bool): True only when no blocker to an ordinary position request was observed.
            This does not determine whether recovery can be attempted.
        side (ArmSide): Selects one of the two physical arms.
        state (ArmState): Arm feedback and command echo.
        warnings (list[str]): Optional diagnostics unavailable or incomplete; not hidden as success.
        advisory (Advisory | None | Unset):
        controller_servos (ArmControllerServos | None | Unset):
    """

    blocking: list[str]
    commanded_gap_deg: float
    feedback_advancing: bool
    initial_frame_serial: int
    max_velocity_deg_s: float
    ready: bool
    side: ArmSide
    state: ArmState
    warnings: list[str]
    advisory: Advisory | None | Unset = UNSET
    controller_servos: ArmControllerServos | None | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        from ..models.advisory import Advisory
        from ..models.arm_controller_servos import ArmControllerServos

        blocking = self.blocking

        commanded_gap_deg = self.commanded_gap_deg

        feedback_advancing = self.feedback_advancing

        initial_frame_serial = self.initial_frame_serial

        max_velocity_deg_s = self.max_velocity_deg_s

        ready = self.ready

        side = self.side.value

        state = self.state.to_dict()

        warnings = self.warnings

        advisory: dict[str, Any] | None | Unset
        if isinstance(self.advisory, Unset):
            advisory = UNSET
        elif isinstance(self.advisory, Advisory):
            advisory = self.advisory.to_dict()
        else:
            advisory = self.advisory

        controller_servos: dict[str, Any] | None | Unset
        if isinstance(self.controller_servos, Unset):
            controller_servos = UNSET
        elif isinstance(self.controller_servos, ArmControllerServos):
            controller_servos = self.controller_servos.to_dict()
        else:
            controller_servos = self.controller_servos

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "blocking": blocking,
                "commanded_gap_deg": commanded_gap_deg,
                "feedback_advancing": feedback_advancing,
                "initial_frame_serial": initial_frame_serial,
                "max_velocity_deg_s": max_velocity_deg_s,
                "ready": ready,
                "side": side,
                "state": state,
                "warnings": warnings,
            }
        )
        if advisory is not UNSET:
            field_dict["advisory"] = advisory
        if controller_servos is not UNSET:
            field_dict["controller_servos"] = controller_servos

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.advisory import Advisory
        from ..models.arm_controller_servos import ArmControllerServos
        from ..models.arm_state import ArmState

        d = dict(src_dict)
        blocking = cast(list[str], d.pop("blocking"))

        commanded_gap_deg = d.pop("commanded_gap_deg")

        feedback_advancing = d.pop("feedback_advancing")

        initial_frame_serial = d.pop("initial_frame_serial")

        max_velocity_deg_s = d.pop("max_velocity_deg_s")

        ready = d.pop("ready")

        side = ArmSide(d.pop("side"))

        state = ArmState.from_dict(d.pop("state"))

        warnings = cast(list[str], d.pop("warnings"))

        def _parse_advisory(data: object) -> Advisory | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            try:
                if not isinstance(data, dict):
                    raise TypeError()
                advisory_type_1 = Advisory.from_dict(data)

                return advisory_type_1
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            return cast(Advisory | None | Unset, data)

        advisory = _parse_advisory(d.pop("advisory", UNSET))

        def _parse_controller_servos(
            data: object,
        ) -> ArmControllerServos | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            try:
                if not isinstance(data, dict):
                    raise TypeError()
                controller_servos_type_1 = ArmControllerServos.from_dict(data)

                return controller_servos_type_1
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            return cast(ArmControllerServos | None | Unset, data)

        controller_servos = _parse_controller_servos(d.pop("controller_servos", UNSET))

        arm_preflight_report = cls(
            blocking=blocking,
            commanded_gap_deg=commanded_gap_deg,
            feedback_advancing=feedback_advancing,
            initial_frame_serial=initial_frame_serial,
            max_velocity_deg_s=max_velocity_deg_s,
            ready=ready,
            side=side,
            state=state,
            warnings=warnings,
            advisory=advisory,
            controller_servos=controller_servos,
        )

        arm_preflight_report.additional_properties = d
        return arm_preflight_report

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
