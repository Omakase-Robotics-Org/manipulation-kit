from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.error_kind import ErrorKind
from ..models.ws_update_event_name import WsUpdateEventName
from ..types import UNSET, Unset

T = TypeVar("T", bound="WsUpdateEvent")


@_attrs_define
class WsUpdateEvent:
    """One sample of a subscription, pushed by the server.

    The first one follows the `subscribe` response immediately. After that a
    frame is sent only when the value CHANGES: consecutive identical samples
    are dropped, so silence means the reading has not moved. There is no
    heartbeat, because the connection itself carries WebSocket ping/pong and
    its liveness is observable there.

    A sample whose read FAILED is pushed like any other, with `data` null and
    the failure in `message`/`code`/`kind`, and the subscription stays alive:
    a device that is unreachable for one sample is expected to come back, and
    the consumer would otherwise have to resubscribe to find out that it did.

        Attributes:
            data (Any): The value the method answered with, or null when the read failed.
            event (WsUpdateEventName): The `event` discriminator of [`WsUpdateEvent`].
            method (str): The method being sampled, echoed from the subscription.
            subscription (int): The subscription this sample belongs to.
            code (int | None | Unset): The mobile base's own numeric response code when the failure came from
                the base, null otherwise.
            kind (ErrorKind | None | Unset):
            message (None | str | Unset): The failure reason when the read failed, null otherwise.
    """

    data: Any
    event: WsUpdateEventName
    method: str
    subscription: int
    code: int | None | Unset = UNSET
    kind: ErrorKind | None | Unset = UNSET
    message: None | str | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        data = self.data

        event = self.event.value

        method = self.method

        subscription = self.subscription

        code: int | None | Unset
        if isinstance(self.code, Unset):
            code = UNSET
        else:
            code = self.code

        kind: None | str | Unset
        if isinstance(self.kind, Unset):
            kind = UNSET
        elif isinstance(self.kind, ErrorKind):
            kind = self.kind.value
        else:
            kind = self.kind

        message: None | str | Unset
        if isinstance(self.message, Unset):
            message = UNSET
        else:
            message = self.message

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "data": data,
                "event": event,
                "method": method,
                "subscription": subscription,
            }
        )
        if code is not UNSET:
            field_dict["code"] = code
        if kind is not UNSET:
            field_dict["kind"] = kind
        if message is not UNSET:
            field_dict["message"] = message

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        data = d.pop("data")

        event = WsUpdateEventName(d.pop("event"))

        method = d.pop("method")

        subscription = d.pop("subscription")

        def _parse_code(data: object) -> int | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(int | None | Unset, data)

        code = _parse_code(d.pop("code", UNSET))

        def _parse_kind(data: object) -> ErrorKind | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            try:
                if not isinstance(data, str):
                    raise TypeError()
                kind_type_1 = ErrorKind(data)

                return kind_type_1
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            return cast(ErrorKind | None | Unset, data)

        kind = _parse_kind(d.pop("kind", UNSET))

        def _parse_message(data: object) -> None | str | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(None | str | Unset, data)

        message = _parse_message(d.pop("message", UNSET))

        ws_update_event = cls(
            data=data,
            event=event,
            method=method,
            subscription=subscription,
            code=code,
            kind=kind,
            message=message,
        )

        ws_update_event.additional_properties = d
        return ws_update_event

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
