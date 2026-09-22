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
    from ..models.chassis_charge_state import ChassisChargeState


T = TypeVar("T", bound="ChassisChargeGetResponse200")


@_attrs_define
class ChassisChargeGetResponse200:
    """
    Attributes:
        data (ChassisChargeState): Everything the daemon knows about the mobile base's charging, in one
            object: where the base is in its docking routine, whether automatic
            charging is the mode it is in, what its battery packs say about current
            flow, and which waypoint pressing "charge" would send it to.

            The four are separate fields because they are four different measurements
            that routinely disagree, and folding them into one flag is what makes an
            operator guess. The vendor's own sequence, from its example program
            `7.add_charge.py`, is: start automatic charging, and `workMode` goes 1 to 2
            while `dockStatus` goes 0 to 1; docking completes and `dockStatus` becomes
            2; the base leaves and `dockStatus` goes 3 then 0 while `workMode` returns
            to 1. Through all of that, whether current is actually flowing is reported
            only by the battery pack.
        message (None): Always null on success.
        status (Literal['ok']):
    """

    data: ChassisChargeState
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
        from ..models.chassis_charge_state import ChassisChargeState

        d = dict(src_dict)
        data = ChassisChargeState.from_dict(d.pop("data"))

        message = d.pop("message")

        status = cast(Literal["ok"], d.pop("status"))
        if status != "ok":
            raise ValueError(f"status must match const 'ok', got '{status}'")

        chassis_charge_get_response_200 = cls(
            data=data,
            message=message,
            status=status,
        )

        chassis_charge_get_response_200.additional_properties = d
        return chassis_charge_get_response_200

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
