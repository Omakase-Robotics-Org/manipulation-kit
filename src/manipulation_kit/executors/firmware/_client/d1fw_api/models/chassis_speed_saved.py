from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.chassis_application_state import ChassisApplicationState
from ..models.chassis_speed_readback_scope import ChassisSpeedReadbackScope

T = TypeVar("T", bound="ChassisSpeedSaved")


@_attrs_define
class ChassisSpeedSaved:
    """Acknowledgment and aggregate saved readback; never physical application.

    Attributes:
        accepted (bool): Manufacturer acknowledged the request, not proof every file was written.
        application_state (ChassisApplicationState): What can be proven about application to the running controller.
        readback_matches (bool): Aggregate getSpeedSetting record matches the proposal and retained fields.
        readback_scope (ChassisSpeedReadbackScope): The limited source observed after an accepted profile save.
        value (Any): Complete manufacturer record received after acceptance.
    """

    accepted: bool
    application_state: ChassisApplicationState
    readback_matches: bool
    readback_scope: ChassisSpeedReadbackScope
    value: Any
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        accepted = self.accepted

        application_state = self.application_state.value

        readback_matches = self.readback_matches

        readback_scope = self.readback_scope.value

        value = self.value

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "accepted": accepted,
                "application_state": application_state,
                "readback_matches": readback_matches,
                "readback_scope": readback_scope,
                "value": value,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        accepted = d.pop("accepted")

        application_state = ChassisApplicationState(d.pop("application_state"))

        readback_matches = d.pop("readback_matches")

        readback_scope = ChassisSpeedReadbackScope(d.pop("readback_scope"))

        value = d.pop("value")

        chassis_speed_saved = cls(
            accepted=accepted,
            application_state=application_state,
            readback_matches=readback_matches,
            readback_scope=readback_scope,
            value=value,
        )

        chassis_speed_saved.additional_properties = d
        return chassis_speed_saved

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
