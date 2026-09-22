from __future__ import annotations

from collections.abc import Mapping
from typing import (
    TYPE_CHECKING,
    Any,
    Literal,
    TypeVar,
    cast,
)

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

if TYPE_CHECKING:
    from ..models.chassis_speed_saved import ChassisSpeedSaved


T = TypeVar("T", bound="ChassisSpeedSettingsSaveResponse200")


@_attrs_define
class ChassisSpeedSettingsSaveResponse200:
    """
    Attributes:
        data (ChassisSpeedSaved): Acknowledgment and aggregate saved readback; never physical application.
        message (None): Always null on success.
        status (Literal['ok']):
    """

    data: ChassisSpeedSaved
    message: None
    status: Literal["ok"]
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        data = self.data.to_dict()

        message = self.message

        status = self.status

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "data": data,
                "message": message,
                "status": status,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.chassis_speed_saved import ChassisSpeedSaved

        d = dict(src_dict)
        data = ChassisSpeedSaved.from_dict(d.pop("data"))

        message = d.pop("message")

        status = cast(Literal["ok"], d.pop("status"))
        if status != "ok":
            raise ValueError(f"status must match const 'ok', got '{status}'")

        chassis_speed_settings_save_response_200 = cls(
            data=data,
            message=message,
            status=status,
        )

        chassis_speed_settings_save_response_200.additional_properties = d
        return chassis_speed_settings_save_response_200

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
