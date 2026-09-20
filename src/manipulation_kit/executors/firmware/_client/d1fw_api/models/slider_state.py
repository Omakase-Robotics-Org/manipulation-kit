from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

T = TypeVar("T", bound="SliderState")


@_attrs_define
class SliderState:
    """Slider feedback.

    Attributes:
        alarm (bool): Whether the slider reports an alarm.
        alarm_text (str): Human-readable alarm text.
        comms_ok (bool): Whether communications with the slider are healthy.
        height_m (float): Current height in metres.
        moving (bool): Whether the slider is moving.
    """

    alarm: bool
    alarm_text: str
    comms_ok: bool
    height_m: float
    moving: bool
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        alarm = self.alarm

        alarm_text = self.alarm_text

        comms_ok = self.comms_ok

        height_m = self.height_m

        moving = self.moving

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "alarm": alarm,
                "alarm_text": alarm_text,
                "comms_ok": comms_ok,
                "height_m": height_m,
                "moving": moving,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        alarm = d.pop("alarm")

        alarm_text = d.pop("alarm_text")

        comms_ok = d.pop("comms_ok")

        height_m = d.pop("height_m")

        moving = d.pop("moving")

        slider_state = cls(
            alarm=alarm,
            alarm_text=alarm_text,
            comms_ok=comms_ok,
            height_m=height_m,
            moving=moving,
        )

        slider_state.additional_properties = d
        return slider_state

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
