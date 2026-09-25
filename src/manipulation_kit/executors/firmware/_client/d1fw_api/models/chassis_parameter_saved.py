from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.chassis_application_state import ChassisApplicationState

T = TypeVar("T", bound="ChassisParameterSaved")


@_attrs_define
class ChassisParameterSaved:
    """Fresh text after the vendor acknowledged a write.

    Attributes:
        application_state (ChassisApplicationState): What can be proven about application to the running controller.
        path (str): Exact inventory path.
        readback_matches (bool): Fresh text exactly equals the submitted text.
        saved (bool): The vendor acknowledged the write.
        text (str): Fresh text after save.
    """

    application_state: ChassisApplicationState
    path: str
    readback_matches: bool
    saved: bool
    text: str
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        application_state = self.application_state.value

        path = self.path

        readback_matches = self.readback_matches

        saved = self.saved

        text = self.text

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "application_state": application_state,
                "path": path,
                "readback_matches": readback_matches,
                "saved": saved,
                "text": text,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        application_state = ChassisApplicationState(d.pop("application_state"))

        path = d.pop("path")

        readback_matches = d.pop("readback_matches")

        saved = d.pop("saved")

        text = d.pop("text")

        chassis_parameter_saved = cls(
            application_state=application_state,
            path=path,
            readback_matches=readback_matches,
            saved=saved,
            text=text,
        )

        chassis_parameter_saved.additional_properties = d
        return chassis_parameter_saved

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
