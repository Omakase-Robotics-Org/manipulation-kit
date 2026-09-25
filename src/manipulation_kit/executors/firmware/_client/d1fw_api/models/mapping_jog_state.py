from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.mapping_jog_delivery import MappingJogDelivery
from ..models.mapping_jog_direction import MappingJogDirection
from ..models.mapping_jog_phase import MappingJogPhase
from ..models.vendor_ros_physical_completion import VendorRosPhysicalCompletion

T = TypeVar("T", bound="MappingJogState")


@_attrs_define
class MappingJogState:
    """Latest bounded lease and command-delivery state.

    Attributes:
        admissible (bool): Whether this daemon's standing preconditions for a mapping jog are
            met, so that a start is worth attempting at all.

            `false` means a start WILL be refused, whatever the operator does at
            the console: the configuration or the mobile base's firmware
            generation rules it out, and nothing about the jog request can change
            that. [`Self::inadmissible_reason`] says which condition failed.

            `true` is not a promise that a start succeeds. It means only that
            these standing conditions hold; the momentary ones are checked at the
            request -- the manufacturer ROS topics must be fresh and consistent
            with the named scene, an outstanding [`Self::stop_required`] must be
            cleared first, and the lease and the stopping epoch are re-checked
            before every send.
        angular_max_rad_s (float): Application angular ceiling, not a proven hardware limit.
        control_token (str): Opaque current admission revision; changes at stopping intent and daemon restart.
        direction (MappingJogDirection | None):
        inadmissible_reason (None | str): Which standing precondition rules a mapping jog out, or `null` when
            [`Self::admissible`] is true.
        lease_id (None | str): Current or last lease identity, absent before first start.
        lease_ms (int): Maximum time granted by one renewal.
        linear_max_m_s (float): Application linear ceiling, not a proven hardware limit.
        motion_delivery (MappingJogDelivery): Delivery evidence for one HTTP command.
        phase (MappingJogPhase): Lifecycle of this daemon's local intent, never a claim about physical stopping.
        physical_completion (VendorRosPhysicalCompletion): Physical completion cannot be determined by a ROS service
            acknowledgment.
        reason (str): Current state/refusal/uncertainty explanation.
        remaining_ms (int): Remaining local hold time; zero outside active phase.
        scene (None | str): Fixed scene of current/last hold.
        speed (float | None): Requested magnitude of current/last hold.
        stop_required (bool): Requires an explicit successful zero request before new local intent.
        zero_delivery (MappingJogDelivery): Delivery evidence for one HTTP command.
    """

    admissible: bool
    angular_max_rad_s: float
    control_token: str
    direction: MappingJogDirection | None
    inadmissible_reason: None | str
    lease_id: None | str
    lease_ms: int
    linear_max_m_s: float
    motion_delivery: MappingJogDelivery
    phase: MappingJogPhase
    physical_completion: VendorRosPhysicalCompletion
    reason: str
    remaining_ms: int
    scene: None | str
    speed: float | None
    stop_required: bool
    zero_delivery: MappingJogDelivery
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        admissible = self.admissible

        angular_max_rad_s = self.angular_max_rad_s

        control_token = self.control_token

        direction: None | str
        if isinstance(self.direction, MappingJogDirection):
            direction = self.direction.value
        else:
            direction = self.direction

        inadmissible_reason: None | str
        inadmissible_reason = self.inadmissible_reason

        lease_id: None | str
        lease_id = self.lease_id

        lease_ms = self.lease_ms

        linear_max_m_s = self.linear_max_m_s

        motion_delivery = self.motion_delivery.value

        phase = self.phase.value

        physical_completion = self.physical_completion.value

        reason = self.reason

        remaining_ms = self.remaining_ms

        scene: None | str
        scene = self.scene

        speed: float | None
        speed = self.speed

        stop_required = self.stop_required

        zero_delivery = self.zero_delivery.value

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "admissible": admissible,
                "angular_max_rad_s": angular_max_rad_s,
                "control_token": control_token,
                "direction": direction,
                "inadmissible_reason": inadmissible_reason,
                "lease_id": lease_id,
                "lease_ms": lease_ms,
                "linear_max_m_s": linear_max_m_s,
                "motion_delivery": motion_delivery,
                "phase": phase,
                "physical_completion": physical_completion,
                "reason": reason,
                "remaining_ms": remaining_ms,
                "scene": scene,
                "speed": speed,
                "stop_required": stop_required,
                "zero_delivery": zero_delivery,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        admissible = d.pop("admissible")

        angular_max_rad_s = d.pop("angular_max_rad_s")

        control_token = d.pop("control_token")

        def _parse_direction(data: object) -> MappingJogDirection | None:
            if data is None:
                return data
            try:
                if not isinstance(data, str):
                    raise TypeError()
                direction_type_1 = MappingJogDirection(data)

                return direction_type_1
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            return cast(MappingJogDirection | None, data)

        direction = _parse_direction(d.pop("direction"))

        def _parse_inadmissible_reason(data: object) -> None | str:
            if data is None:
                return data
            return cast(None | str, data)

        inadmissible_reason = _parse_inadmissible_reason(d.pop("inadmissible_reason"))

        def _parse_lease_id(data: object) -> None | str:
            if data is None:
                return data
            return cast(None | str, data)

        lease_id = _parse_lease_id(d.pop("lease_id"))

        lease_ms = d.pop("lease_ms")

        linear_max_m_s = d.pop("linear_max_m_s")

        motion_delivery = MappingJogDelivery(d.pop("motion_delivery"))

        phase = MappingJogPhase(d.pop("phase"))

        physical_completion = VendorRosPhysicalCompletion(d.pop("physical_completion"))

        reason = d.pop("reason")

        remaining_ms = d.pop("remaining_ms")

        def _parse_scene(data: object) -> None | str:
            if data is None:
                return data
            return cast(None | str, data)

        scene = _parse_scene(d.pop("scene"))

        def _parse_speed(data: object) -> float | None:
            if data is None:
                return data
            return cast(float | None, data)

        speed = _parse_speed(d.pop("speed"))

        stop_required = d.pop("stop_required")

        zero_delivery = MappingJogDelivery(d.pop("zero_delivery"))

        mapping_jog_state = cls(
            admissible=admissible,
            angular_max_rad_s=angular_max_rad_s,
            control_token=control_token,
            direction=direction,
            inadmissible_reason=inadmissible_reason,
            lease_id=lease_id,
            lease_ms=lease_ms,
            linear_max_m_s=linear_max_m_s,
            motion_delivery=motion_delivery,
            phase=phase,
            physical_completion=physical_completion,
            reason=reason,
            remaining_ms=remaining_ms,
            scene=scene,
            speed=speed,
            stop_required=stop_required,
            zero_delivery=zero_delivery,
        )

        mapping_jog_state.additional_properties = d
        return mapping_jog_state

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
