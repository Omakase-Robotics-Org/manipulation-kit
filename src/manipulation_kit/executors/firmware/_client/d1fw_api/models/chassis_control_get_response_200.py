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
    from ..models.chassis_control_state import ChassisControlState


T = TypeVar("T", bound="ChassisControlGetResponse200")


@_attrs_define
class ChassisControlGetResponse200:
    """
    Attributes:
        data (ChassisControlState): Everything the daemon knows about WHICH CONTROLLER may drive the mobile
            base: the mode it is in, what that mode permits, and which control verbs
            are legal from it.

            The mobile base has exactly one control mode at a time and every motion
            path is gated on it, but the vendor publishes that only as a bare
            `workMode` digit and enforces it by IGNORING commands: outside its
            remote-control mode the base answers a jog with HTTP 200 and does not
            move. This object is the daemon's answer to "what state is the base in,
            what can I do from here, and why not" in one read, so that an operator
            interface never has to infer it from a digit or from a command that
            silently did nothing.

            The five vendor modes are the vendor's own, taken from its web
            application's own translation table: `0` navigation closed, `1`
            navigation, `2` automatic charging, `3` mapping, `4` remote control.
        message (None): Always null on success.
        status (Literal['ok']):
    """

    data: ChassisControlState
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
        from ..models.chassis_control_state import ChassisControlState

        d = dict(src_dict)
        data = ChassisControlState.from_dict(d.pop("data"))

        message = d.pop("message")

        status = cast(Literal["ok"], d.pop("status"))
        if status != "ok":
            raise ValueError(f"status must match const 'ok', got '{status}'")

        chassis_control_get_response_200 = cls(
            data=data,
            message=message,
            status=status,
        )

        chassis_control_get_response_200.additional_properties = d
        return chassis_control_get_response_200

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
