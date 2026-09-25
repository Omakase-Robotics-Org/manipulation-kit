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
    from ..models.arm_tool_status import ArmToolStatus


T = TypeVar("T", bound="ArmToolStateResponse200")


@_attrs_define
class ArmToolStateResponse200:
    """
    Attributes:
        data (ArmToolStatus): What the daemon has registered on one arm, and where it came from.

            The arm controller does not report its registered tool in feedback, so
            this is the daemon's own record of what it last sent. It is the answer to
            "is this arm compensating for the gripper it is actually wearing?", which
            is otherwise only visible as a sagging wrist.
        message (None): Always null on success.
        status (Literal['ok']):
    """

    data: ArmToolStatus
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
        from ..models.arm_tool_status import ArmToolStatus

        d = dict(src_dict)
        data = ArmToolStatus.from_dict(d.pop("data"))

        message = d.pop("message")

        status = cast(Literal["ok"], d.pop("status"))
        if status != "ok":
            raise ValueError(f"status must match const 'ok', got '{status}'")

        arm_tool_state_response_200 = cls(
            data=data,
            message=message,
            status=status,
        )

        arm_tool_state_response_200.additional_properties = d
        return arm_tool_state_response_200

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
