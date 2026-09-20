from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..types import UNSET, Unset

T = TypeVar("T", bound="WsRequest")


@_attrs_define
class WsRequest:
    """One request frame sent by a WebSocket client.

    Attributes:
        id (int): Caller-chosen correlation identifier, echoed in the response.

            Responses can arrive out of order (see [`WsResponse`]), so this is the
            only way to match a response to its request.
        method (str): The `x-ws-method` value of the operation to call, for example
            `arm.a.move_joints`.  Side-templated operations are spelled with a
            concrete side: `arm.a.mode`, never `arm.{side}.mode`.
        params (Any | Unset): The operation's request body, as an object.  Omitted means `{}`.
    """

    id: int
    method: str
    params: Any | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        id = self.id

        method = self.method

        params = self.params

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "id": id,
                "method": method,
            }
        )
        if params is not UNSET:
            field_dict["params"] = params

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        id = d.pop("id")

        method = d.pop("method")

        params = d.pop("params", UNSET)

        ws_request = cls(
            id=id,
            method=method,
            params=params,
        )

        ws_request.additional_properties = d
        return ws_request

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
