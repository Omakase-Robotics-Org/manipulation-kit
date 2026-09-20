from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

if TYPE_CHECKING:
    from ..models.hand_state import HandState
    from ..models.slot_error import SlotError


T = TypeVar("T", bound="SnapshotHand")


@_attrs_define
class SnapshotHand:
    """The dexterous hands, keyed by arm side (`"a"` / `"b"`).

    Only the sides that actually CARRY a hand appear, so a robot with a
    gripper on both sides — which is what the D1 ships as — reports an
    empty object rather than two errors for hardware nobody fitted.

    """

    additional_properties: dict[str, HandState | SlotError] = _attrs_field(
        init=False, factory=dict
    )

    def to_dict(self) -> dict[str, Any]:
        from ..models.hand_state import HandState

        field_dict: dict[str, Any] = {}
        for prop_name, prop in self.additional_properties.items():
            if isinstance(prop, HandState):
                field_dict[prop_name] = prop.to_dict()
            else:
                field_dict[prop_name] = prop.to_dict()

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.hand_state import HandState
        from ..models.slot_error import SlotError

        d = dict(src_dict)
        snapshot_hand = cls()

        additional_properties = {}
        for prop_name, prop_dict in d.items():

            def _parse_additional_property(data: object) -> HandState | SlotError:
                try:
                    if not isinstance(data, dict):
                        raise TypeError()
                    componentsschemas_hand_slot_type_0 = HandState.from_dict(data)

                    return componentsschemas_hand_slot_type_0
                except (TypeError, ValueError, AttributeError, KeyError):
                    pass
                if not isinstance(data, dict):
                    raise TypeError()
                componentsschemas_hand_slot_type_1 = SlotError.from_dict(data)

                return componentsschemas_hand_slot_type_1

            additional_property = _parse_additional_property(prop_dict)

            additional_properties[prop_name] = additional_property

        snapshot_hand.additional_properties = additional_properties
        return snapshot_hand

    @property
    def additional_keys(self) -> list[str]:
        return list(self.additional_properties.keys())

    def __getitem__(self, key: str) -> HandState | SlotError:
        return self.additional_properties[key]

    def __setitem__(self, key: str, value: HandState | SlotError) -> None:
        self.additional_properties[key] = value

    def __delitem__(self, key: str) -> None:
        del self.additional_properties[key]

    def __contains__(self, key: str) -> bool:
        return key in self.additional_properties
