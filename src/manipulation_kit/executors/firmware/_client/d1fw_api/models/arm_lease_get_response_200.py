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
    from ..models.arm_lease import ArmLease


T = TypeVar("T", bound="ArmLeaseGetResponse200")


@_attrs_define
class ArmLeaseGetResponse200:
    """
    Attributes:
        data (ArmLease | None):
        message (None): Always null on success.
        status (Literal['ok']):
    """

    data: ArmLease | None
    message: None
    status: Literal["ok"]
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        from ..models.arm_lease import ArmLease

        data: dict[str, Any] | None
        if isinstance(self.data, ArmLease):
            data = self.data.to_dict()
        else:
            data = self.data

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
        from ..models.arm_lease import ArmLease

        d = dict(src_dict)

        def _parse_data(data: object) -> ArmLease | None:
            if data is None:
                return data
            try:
                if not isinstance(data, dict):
                    raise TypeError()
                data_type_0 = ArmLease.from_dict(data)

                return data_type_0
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            return cast(ArmLease | None, data)

        data = _parse_data(d.pop("data"))

        message = d.pop("message")

        status = cast(Literal["ok"], d.pop("status"))
        if status != "ok":
            raise ValueError(f"status must match const 'ok', got '{status}'")

        arm_lease_get_response_200 = cls(
            data=data,
            message=message,
            status=status,
        )

        arm_lease_get_response_200.additional_properties = d
        return arm_lease_get_response_200

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
