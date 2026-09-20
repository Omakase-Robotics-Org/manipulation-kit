from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.device import Device
from ..models.ws_soft_kill_event_name import WsSoftKillEventName

if TYPE_CHECKING:
    from ..models.ws_soft_kill_payload import WsSoftKillPayload


T = TypeVar("T", bound="WsSoftKillEvent")


@_attrs_define
class WsSoftKillEvent:
    """An unsolicited frame pushed when one device's soft-kill latch changes.

    It has no `id`: it answers no request.  Distinguish pushes from responses
    by the presence of `event`.

        Attributes:
            data (WsSoftKillPayload): The payload of a [`WsSoftKillEvent`].
            device (Device): A D1 device for kill-latch and error reporting purposes.
            event (WsSoftKillEventName): The `event` discriminator of [`WsSoftKillEvent`].
    """

    data: WsSoftKillPayload
    device: Device
    event: WsSoftKillEventName
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        data = self.data.to_dict()

        device = self.device.value

        event = self.event.value

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "data": data,
                "device": device,
                "event": event,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.ws_soft_kill_payload import WsSoftKillPayload

        d = dict(src_dict)
        data = WsSoftKillPayload.from_dict(d.pop("data"))

        device = Device(d.pop("device"))

        event = WsSoftKillEventName(d.pop("event"))

        ws_soft_kill_event = cls(
            data=data,
            device=device,
            event=event,
        )

        ws_soft_kill_event.additional_properties = d
        return ws_soft_kill_event

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
