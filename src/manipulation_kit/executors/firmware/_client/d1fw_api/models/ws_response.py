from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.ws_status import WsStatus
from ..types import UNSET, Unset

T = TypeVar("T", bound="WsResponse")


@_attrs_define
class WsResponse:
    """One response frame, carrying the same payload the REST envelope would.

    **Responses are not ordered.**  Each request is dispatched on its own
    task, so a long motion call never delays another request on the same
    connection and a later request can answer first.  Correlate by `id`, never
    by arrival order.

        Attributes:
            data (Any): The operation's result on success, null on error.  This is the same
                payload as the REST envelope's `data` for the same method.
            status (WsStatus): The `status` field of a [`WsResponse`].
            id (int | None | Unset): The `id` of the request this answers, or null when the frame could not
                be parsed far enough to recover one.
            message (None | str | Unset): The failure reason on error, null on success.
    """

    data: Any
    status: WsStatus
    id: int | None | Unset = UNSET
    message: None | str | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        data = self.data

        status = self.status.value

        id: int | None | Unset
        if isinstance(self.id, Unset):
            id = UNSET
        else:
            id = self.id

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
                "status": status,
            }
        )
        if id is not UNSET:
            field_dict["id"] = id
        if message is not UNSET:
            field_dict["message"] = message

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        data = d.pop("data")

        status = WsStatus(d.pop("status"))

        def _parse_id(data: object) -> int | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(int | None | Unset, data)

        id = _parse_id(d.pop("id", UNSET))

        def _parse_message(data: object) -> None | str | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(None | str | Unset, data)

        message = _parse_message(d.pop("message", UNSET))

        ws_response = cls(
            data=data,
            status=status,
            id=id,
            message=message,
        )

        ws_response.additional_properties = d
        return ws_response

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
