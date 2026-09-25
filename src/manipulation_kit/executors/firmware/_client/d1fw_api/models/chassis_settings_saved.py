from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.chassis_application_state import ChassisApplicationState

T = TypeVar("T", bound="ChassisSettingsSaved")


@_attrs_define
class ChassisSettingsSaved:
    """Saving a record verifies storage only, never runtime application.

    Attributes:
        application_state (ChassisApplicationState): What can be proven about application to the running controller.
        readback_matches (bool): Fresh read equals the requested merged record.
        saved (bool): Vendor acknowledged the save; this is not proof of runtime application.
        value (Any): Fresh vendor read after the acknowledged write.
    """

    application_state: ChassisApplicationState
    readback_matches: bool
    saved: bool
    value: Any
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        application_state = self.application_state.value

        readback_matches = self.readback_matches

        saved = self.saved

        value = self.value

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "application_state": application_state,
                "readback_matches": readback_matches,
                "saved": saved,
                "value": value,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        application_state = ChassisApplicationState(d.pop("application_state"))

        readback_matches = d.pop("readback_matches")

        saved = d.pop("saved")

        value = d.pop("value")

        chassis_settings_saved = cls(
            application_state=application_state,
            readback_matches=readback_matches,
            saved=saved,
            value=value,
        )

        chassis_settings_saved.additional_properties = d
        return chassis_settings_saved

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
