from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..types import UNSET, Unset

if TYPE_CHECKING:
    from ..models.arm_lease import ArmLease
    from ..models.arm_state import ArmState
    from ..models.slot_error import SlotError


T = TypeVar("T", bound="ArmSlots")


@_attrs_define
class ArmSlots:
    """The two arms' slots, keyed by side, and the operator lease over both.

    Attributes:
        a (ArmState | SlotError):
            Either the device's own state or, when that device could not be
            read, a [`SlotError`] naming the failure.
        b (ArmState | SlotError):
            Either the device's own state or, when that device could not be
            read, a [`SlotError`] naming the failure.
        lease (ArmLease | None | Unset):
    """

    a: ArmState | SlotError
    b: ArmState | SlotError
    lease: ArmLease | None | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        from ..models.arm_lease import ArmLease
        from ..models.arm_state import ArmState

        a: dict[str, Any]
        if isinstance(self.a, ArmState):
            a = self.a.to_dict()
        else:
            a = self.a.to_dict()

        b: dict[str, Any]
        if isinstance(self.b, ArmState):
            b = self.b.to_dict()
        else:
            b = self.b.to_dict()

        lease: dict[str, Any] | None | Unset
        if isinstance(self.lease, Unset):
            lease = UNSET
        elif isinstance(self.lease, ArmLease):
            lease = self.lease.to_dict()
        else:
            lease = self.lease

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "a": a,
                "b": b,
            }
        )
        if lease is not UNSET:
            field_dict["lease"] = lease

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.arm_lease import ArmLease
        from ..models.arm_state import ArmState
        from ..models.slot_error import SlotError

        d = dict(src_dict)

        def _parse_a(data: object) -> ArmState | SlotError:
            try:
                if not isinstance(data, dict):
                    raise TypeError()
                componentsschemas_arm_slot_type_0 = ArmState.from_dict(data)

                return componentsschemas_arm_slot_type_0
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            if not isinstance(data, dict):
                raise TypeError()
            componentsschemas_arm_slot_type_1 = SlotError.from_dict(data)

            return componentsschemas_arm_slot_type_1

        a = _parse_a(d.pop("a"))

        def _parse_b(data: object) -> ArmState | SlotError:
            try:
                if not isinstance(data, dict):
                    raise TypeError()
                componentsschemas_arm_slot_type_0 = ArmState.from_dict(data)

                return componentsschemas_arm_slot_type_0
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            if not isinstance(data, dict):
                raise TypeError()
            componentsschemas_arm_slot_type_1 = SlotError.from_dict(data)

            return componentsschemas_arm_slot_type_1

        b = _parse_b(d.pop("b"))

        def _parse_lease(data: object) -> ArmLease | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            try:
                if not isinstance(data, dict):
                    raise TypeError()
                lease_type_1 = ArmLease.from_dict(data)

                return lease_type_1
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            return cast(ArmLease | None | Unset, data)

        lease = _parse_lease(d.pop("lease", UNSET))

        arm_slots = cls(
            a=a,
            b=b,
            lease=lease,
        )

        arm_slots.additional_properties = d
        return arm_slots

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
