from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

T = TypeVar("T", bound="WsSubscribeResult")


@_attrs_define
class WsSubscribeResult:
    """The `data` of the response to `subscribe`.

    Attributes:
        interval_ms (int): The sampling interval actually in force, after clamping.
        method (str): The method that was subscribed to, echoed.
        subscription (int): The identifier every [`WsUpdateEvent`] of this subscription carries,
            and the one to pass to `unsubscribe`.

            Per-connection and starting at 1: it means nothing on another
            connection, and a reconnecting client counts from 1 again.
    """

    interval_ms: int
    method: str
    subscription: int
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        interval_ms = self.interval_ms

        method = self.method

        subscription = self.subscription

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "interval_ms": interval_ms,
                "method": method,
                "subscription": subscription,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        interval_ms = d.pop("interval_ms")

        method = d.pop("method")

        subscription = d.pop("subscription")

        ws_subscribe_result = cls(
            interval_ms=interval_ms,
            method=method,
            subscription=subscription,
        )

        ws_subscribe_result.additional_properties = d
        return ws_subscribe_result

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
