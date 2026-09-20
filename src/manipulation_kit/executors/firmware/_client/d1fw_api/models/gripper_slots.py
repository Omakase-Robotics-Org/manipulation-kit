from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

if TYPE_CHECKING:
    from ..models.gripper_report import GripperReport
    from ..models.slot_error import SlotError


T = TypeVar("T", bound="GripperSlots")


@_attrs_define
class GripperSlots:
    """The two grippers' slots, keyed by side.

    Attributes:
        a (GripperReport | SlotError):
            Either the device's own state or, when that device could not be
            read, a [`SlotError`] naming the failure.
        b (GripperReport | SlotError):
            Either the device's own state or, when that device could not be
            read, a [`SlotError`] naming the failure.
    """

    a: GripperReport | SlotError
    b: GripperReport | SlotError
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        from ..models.gripper_report import GripperReport

        a: dict[str, Any]
        if isinstance(self.a, GripperReport):
            a = self.a.to_dict()
        else:
            a = self.a.to_dict()

        b: dict[str, Any]
        if isinstance(self.b, GripperReport):
            b = self.b.to_dict()
        else:
            b = self.b.to_dict()

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "a": a,
                "b": b,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.gripper_report import GripperReport
        from ..models.slot_error import SlotError

        d = dict(src_dict)

        def _parse_a(data: object) -> GripperReport | SlotError:
            try:
                if not isinstance(data, dict):
                    raise TypeError()
                componentsschemas_gripper_slot_type_0 = GripperReport.from_dict(data)

                return componentsschemas_gripper_slot_type_0
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            if not isinstance(data, dict):
                raise TypeError()
            componentsschemas_gripper_slot_type_1 = SlotError.from_dict(data)

            return componentsschemas_gripper_slot_type_1

        a = _parse_a(d.pop("a"))

        def _parse_b(data: object) -> GripperReport | SlotError:
            try:
                if not isinstance(data, dict):
                    raise TypeError()
                componentsschemas_gripper_slot_type_0 = GripperReport.from_dict(data)

                return componentsschemas_gripper_slot_type_0
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            if not isinstance(data, dict):
                raise TypeError()
            componentsschemas_gripper_slot_type_1 = SlotError.from_dict(data)

            return componentsschemas_gripper_slot_type_1

        b = _parse_b(d.pop("b"))

        gripper_slots = cls(
            a=a,
            b=b,
        )

        gripper_slots.additional_properties = d
        return gripper_slots

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
