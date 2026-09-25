from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..types import UNSET, Unset

T = TypeVar("T", bound="WsSubscribeParams")


@_attrs_define
class WsSubscribeParams:
    """The `params` of a `subscribe` request.

    A subscription names a read and is answered with that read's value now,
    and again whenever the value changes. It is not a second API: the value
    pushed is exactly what the named method would have answered.

        Attributes:
            method (str): The read to subscribe to, spelled as an `x-ws-method` value with a
                concrete side: `state`, `arm.a.state`, `chassis.state`.

                Only operations whose HTTP verb is `GET` can be subscribed to, and
                only those this frontend dispatches: a method that writes, or one
                marked REST-only, is refused.
            interval_ms (int | None | Unset): How often to sample, in milliseconds.

                Clamped server-side to the method's floor and reported back in
                [`WsSubscribeResult::interval_ms`]; a value below the floor is clamped,
                not refused. `state` and every other read default to 500 ms with a
                floor of 200 ms, because one `state` read costs an HTTP round trip to
                the mobile base plus a serial and a Modbus round trip on the robot.
                `arm.a.state` and `arm.b.state` default to 100 ms with a floor of
                50 ms: they are answered from the arm feedback stream and touch no bus.
            params (Any | Unset): The parameters that read takes, if any. Forwarded unchanged to every
                sample. Omitted means `{}`.
    """

    method: str
    interval_ms: int | None | Unset = UNSET
    params: Any | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        method = self.method

        interval_ms: int | None | Unset
        if isinstance(self.interval_ms, Unset):
            interval_ms = UNSET
        else:
            interval_ms = self.interval_ms

        params = self.params

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "method": method,
            }
        )
        if interval_ms is not UNSET:
            field_dict["interval_ms"] = interval_ms
        if params is not UNSET:
            field_dict["params"] = params

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        method = d.pop("method")

        def _parse_interval_ms(data: object) -> int | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(int | None | Unset, data)

        interval_ms = _parse_interval_ms(d.pop("interval_ms", UNSET))

        params = d.pop("params", UNSET)

        ws_subscribe_params = cls(
            method=method,
            interval_ms=interval_ms,
            params=params,
        )

        ws_subscribe_params.additional_properties = d
        return ws_subscribe_params

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
