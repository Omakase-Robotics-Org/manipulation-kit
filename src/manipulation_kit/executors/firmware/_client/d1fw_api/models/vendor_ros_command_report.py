from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.vendor_ros_outcome import VendorRosOutcome
from ..models.vendor_ros_physical_completion import VendorRosPhysicalCompletion
from ..types import UNSET, Unset

if TYPE_CHECKING:
    from ..models.vendor_mapping_observation import VendorMappingObservation


T = TypeVar("T", bound="VendorRosCommandReport")


@_attrs_define
class VendorRosCommandReport:
    """Receipt from one fixed ROS service or pose publish attempt; never auto-retry.

    For command=pose_reset the only outcomes are transport_sent and outcome_unknown,
    vendor_success is always present and null, physical_completion is unknown and
    mapping_observation is absent. transport_sent means only local WebSocket write
    completion; no ROS subscriber acceptance, reset, convergence or physical completion
    is established. An acknowledged/refused pose report is a contract violation, not
    evidence of success/rejection; treat delivery as unknown. Service commands retain
    acknowledged/refused/outcome_unknown and their matching service success flag.

    For pose_reset, a complete authentic daemon error envelope (status=error, data=null,
    kind and message) represents rejection/setup failure before the pose publish attempt.
    Setup may advertise the fixed topic but carries no pose. Once publish send begins,
    failure/timeout returns an outcome_unknown report instead of an ordinary error.
    A missing, malformed, proxy-generated or network response does not establish this
    prepublish guarantee. Request IDs are correlation only; reread and review, never retry.

        Attributes:
            command (str): Command family identifier.
            description (str): Vendor service description or explicit pose transport evidence/uncertainty.
            outcome (VendorRosOutcome): Outcome of a single command attempt, never automatically resent.
            physical_completion (VendorRosPhysicalCompletion): Physical completion cannot be determined by a ROS service
                acknowledgment.
            request_id (str): Unique correlation ID; this is not a vendor idempotency token.
            vendor_success (bool | None): Vendor success flag, null when no well-formed matching reply arrived.
            mapping_observation (None | Unset | VendorMappingObservation):
    """

    command: str
    description: str
    outcome: VendorRosOutcome
    physical_completion: VendorRosPhysicalCompletion
    request_id: str
    vendor_success: bool | None
    mapping_observation: None | Unset | VendorMappingObservation = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        from ..models.vendor_mapping_observation import (
            VendorMappingObservation,
        )

        command = self.command

        description = self.description

        outcome = self.outcome.value

        physical_completion = self.physical_completion.value

        request_id = self.request_id

        vendor_success: bool | None
        vendor_success = self.vendor_success

        mapping_observation: dict[str, Any] | None | Unset
        if isinstance(self.mapping_observation, Unset):
            mapping_observation = UNSET
        elif isinstance(self.mapping_observation, VendorMappingObservation):
            mapping_observation = self.mapping_observation.to_dict()
        else:
            mapping_observation = self.mapping_observation

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "command": command,
                "description": description,
                "outcome": outcome,
                "physical_completion": physical_completion,
                "request_id": request_id,
                "vendor_success": vendor_success,
            }
        )
        if mapping_observation is not UNSET:
            field_dict["mapping_observation"] = mapping_observation

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.vendor_mapping_observation import (
            VendorMappingObservation,
        )

        d = dict(src_dict)
        command = d.pop("command")

        description = d.pop("description")

        outcome = VendorRosOutcome(d.pop("outcome"))

        physical_completion = VendorRosPhysicalCompletion(d.pop("physical_completion"))

        request_id = d.pop("request_id")

        def _parse_vendor_success(data: object) -> bool | None:
            if data is None:
                return data
            return cast(bool | None, data)

        vendor_success = _parse_vendor_success(d.pop("vendor_success"))

        def _parse_mapping_observation(
            data: object,
        ) -> None | Unset | VendorMappingObservation:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            try:
                if not isinstance(data, dict):
                    raise TypeError()
                mapping_observation_type_1 = VendorMappingObservation.from_dict(data)

                return mapping_observation_type_1
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            return cast(None | Unset | VendorMappingObservation, data)

        mapping_observation = _parse_mapping_observation(
            d.pop("mapping_observation", UNSET)
        )

        vendor_ros_command_report = cls(
            command=command,
            description=description,
            outcome=outcome,
            physical_completion=physical_completion,
            request_id=request_id,
            vendor_success=vendor_success,
            mapping_observation=mapping_observation,
        )

        vendor_ros_command_report.additional_properties = d
        return vendor_ros_command_report

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
