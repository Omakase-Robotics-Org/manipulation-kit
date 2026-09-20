from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.ws_nav_event_name import WsNavEventName

if TYPE_CHECKING:
    from ..models.ws_nav_payload import WsNavPayload


T = TypeVar("T", bound="WsNavEvent")


@_attrs_define
class WsNavEvent:
    """An unsolicited frame pushed as a navigation goal progresses.

    Pushed only to the connection that started the goal, until the goal
    reaches a terminal phase.

        Attributes:
            data (WsNavPayload): The payload of a [`WsNavEvent`].
            event (WsNavEventName): The `event` discriminator of [`WsNavEvent`].
    """

    data: WsNavPayload
    event: WsNavEventName
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        data = self.data.to_dict()

        event = self.event.value

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "data": data,
                "event": event,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.ws_nav_payload import WsNavPayload

        d = dict(src_dict)
        data = WsNavPayload.from_dict(d.pop("data"))

        event = WsNavEventName(d.pop("event"))

        ws_nav_event = cls(
            data=data,
            event=event,
        )

        ws_nav_event.additional_properties = d
        return ws_nav_event

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
