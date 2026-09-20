from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.arm_mode_request_type_0_mode import ArmModeRequestType0Mode

T = TypeVar("T", bound="ArmModeRequestType0")


@_attrs_define
class ArmModeRequestType0:
    """Disable position control.

    Attributes:
        mode (ArmModeRequestType0Mode):
    """

    mode: ArmModeRequestType0Mode
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        mode = self.mode.value

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "mode": mode,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        mode = ArmModeRequestType0Mode(d.pop("mode"))

        arm_mode_request_type_0 = cls(
            mode=mode,
        )

        arm_mode_request_type_0.additional_properties = d
        return arm_mode_request_type_0

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
