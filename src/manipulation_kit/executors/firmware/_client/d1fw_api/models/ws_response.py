from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.error_kind import ErrorKind
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
            code (int | None | Unset): The mobile base's own numeric response code, when the failure came
                from the base. Null on a failure raised by the daemon itself, and
                absent from a success frame, which carries the three fields it has
                always carried.

                The same field the REST failure envelope carries, so a consumer that
                speaks both frontends branches on one vocabulary.
            id (int | None | Unset): The `id` of the request this answers, or null when the frame could not
                be parsed far enough to recover one.
            kind (ErrorKind | None | Unset):
            message (None | str | Unset): The failure reason on error, null on success.
    """

    data: Any
    status: WsStatus
    code: int | None | Unset = UNSET
    id: int | None | Unset = UNSET
    kind: ErrorKind | None | Unset = UNSET
    message: None | str | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        data = self.data

        status = self.status.value

        code: int | None | Unset
        if isinstance(self.code, Unset):
            code = UNSET
        else:
            code = self.code

        id: int | None | Unset
        if isinstance(self.id, Unset):
            id = UNSET
        else:
            id = self.id

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
                "status": status,
            }
        )
        if code is not UNSET:
            field_dict["code"] = code
        if id is not UNSET:
            field_dict["id"] = id
        if kind is not UNSET:
            field_dict["kind"] = kind
        if message is not UNSET:
            field_dict["message"] = message

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        data = d.pop("data")

        status = WsStatus(d.pop("status"))

        def _parse_code(data: object) -> int | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(int | None | Unset, data)

        code = _parse_code(d.pop("code", UNSET))

        def _parse_id(data: object) -> int | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(int | None | Unset, data)

        id = _parse_id(d.pop("id", UNSET))

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

        ws_response = cls(
            data=data,
            status=status,
            code=code,
            id=id,
            kind=kind,
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
