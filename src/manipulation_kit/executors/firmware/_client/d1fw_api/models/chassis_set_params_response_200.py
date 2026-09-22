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
    from ..models.chassis_set_params_report import ChassisSetParamsReport


T = TypeVar("T", bound="ChassisSetParamsResponse200")


@_attrs_define
class ChassisSetParamsResponse200:
    """
    Attributes:
        data (ChassisSetParamsReport): What one [`ChassisParam`] write did.

            Three fields exist because a write can be accepted and still change
            nothing, in three independent ways: the older mobile-base firmware
            generation serves `setParams` and ignores what it is told
            ([`ChassisSetParamsReport::effective`]); the vendor writes a YAML file and
            no running node re-reads it, so the value waits for a restart, and for
            WHICH restart depends on the parameter
            ([`ChassisSetParamsReport::applies_when`]); and for one parameter no node
            reads the written file at all. All of it travels with the response rather
            than being left for the caller to know.
        message (None): Always null on success.
        status (Literal['ok']):
    """

    data: ChassisSetParamsReport
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
        from ..models.chassis_set_params_report import (
            ChassisSetParamsReport,
        )

        d = dict(src_dict)
        data = ChassisSetParamsReport.from_dict(d.pop("data"))

        message = d.pop("message")

        status = cast(Literal["ok"], d.pop("status"))
        if status != "ok":
            raise ValueError(f"status must match const 'ok', got '{status}'")

        chassis_set_params_response_200 = cls(
            data=data,
            message=message,
            status=status,
        )

        chassis_set_params_response_200.additional_properties = d
        return chassis_set_params_response_200

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
