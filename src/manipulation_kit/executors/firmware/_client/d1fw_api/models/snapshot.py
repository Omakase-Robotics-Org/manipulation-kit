from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

if TYPE_CHECKING:
    from ..models.arm_slots import ArmSlots
    from ..models.chassis_state import ChassisState
    from ..models.eyes_state import EyesState
    from ..models.gripper_slots import GripperSlots
    from ..models.kill_snapshot import KillSnapshot
    from ..models.neck_state import NeckState
    from ..models.slider_state import SliderState
    from ..models.slot_error import SlotError
    from ..models.snapshot_hand import SnapshotHand


T = TypeVar("T", bound="Snapshot")


@_attrs_define
class Snapshot:
    """One whole-robot snapshot.

    Attributes:
        arm (ArmSlots): The two arms' slots, keyed by side.
        chassis (ChassisState | SlotError):
            Either the device's own state or, when that device could not be
            read, a [`SlotError`] naming the failure.
        eyes (EyesState | SlotError):
            Either the device's own state or, when that device could not be
            read, a [`SlotError`] naming the failure.
        gripper (GripperSlots): The two grippers' slots, keyed by side.
        hand (SnapshotHand): The dexterous hands, keyed by arm side (`"a"` / `"b"`).

            Only the sides that actually CARRY a hand appear, so a robot with a
            gripper on both sides — which is what the D1 ships as — reports an
            empty object rather than two errors for hardware nobody fitted.
        kill (KillSnapshot): A copyable snapshot of every device's soft-kill latch.
        neck (NeckState | SlotError):
            Either the device's own state or, when that device could not be
            read, a [`SlotError`] naming the failure.
        slider (SliderState | SlotError):
            Either the device's own state or, when that device could not be
            read, a [`SlotError`] naming the failure.
    """

    arm: ArmSlots
    chassis: ChassisState | SlotError
    eyes: EyesState | SlotError
    gripper: GripperSlots
    hand: SnapshotHand
    kill: KillSnapshot
    neck: NeckState | SlotError
    slider: SliderState | SlotError
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        from ..models.chassis_state import ChassisState
        from ..models.eyes_state import EyesState
        from ..models.neck_state import NeckState
        from ..models.slider_state import SliderState

        arm = self.arm.to_dict()

        chassis: dict[str, Any]
        if isinstance(self.chassis, ChassisState):
            chassis = self.chassis.to_dict()
        else:
            chassis = self.chassis.to_dict()

        eyes: dict[str, Any]
        if isinstance(self.eyes, EyesState):
            eyes = self.eyes.to_dict()
        else:
            eyes = self.eyes.to_dict()

        gripper = self.gripper.to_dict()

        hand = self.hand.to_dict()

        kill = self.kill.to_dict()

        neck: dict[str, Any]
        if isinstance(self.neck, NeckState):
            neck = self.neck.to_dict()
        else:
            neck = self.neck.to_dict()

        slider: dict[str, Any]
        if isinstance(self.slider, SliderState):
            slider = self.slider.to_dict()
        else:
            slider = self.slider.to_dict()

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "arm": arm,
                "chassis": chassis,
                "eyes": eyes,
                "gripper": gripper,
                "hand": hand,
                "kill": kill,
                "neck": neck,
                "slider": slider,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.arm_slots import ArmSlots
        from ..models.chassis_state import ChassisState
        from ..models.eyes_state import EyesState
        from ..models.gripper_slots import GripperSlots
        from ..models.kill_snapshot import KillSnapshot
        from ..models.neck_state import NeckState
        from ..models.slider_state import SliderState
        from ..models.slot_error import SlotError
        from ..models.snapshot_hand import SnapshotHand

        d = dict(src_dict)
        arm = ArmSlots.from_dict(d.pop("arm"))

        def _parse_chassis(data: object) -> ChassisState | SlotError:
            try:
                if not isinstance(data, dict):
                    raise TypeError()
                componentsschemas_chassis_slot_type_0 = ChassisState.from_dict(data)

                return componentsschemas_chassis_slot_type_0
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            if not isinstance(data, dict):
                raise TypeError()
            componentsschemas_chassis_slot_type_1 = SlotError.from_dict(data)

            return componentsschemas_chassis_slot_type_1

        chassis = _parse_chassis(d.pop("chassis"))

        def _parse_eyes(data: object) -> EyesState | SlotError:
            try:
                if not isinstance(data, dict):
                    raise TypeError()
                componentsschemas_eyes_slot_type_0 = EyesState.from_dict(data)

                return componentsschemas_eyes_slot_type_0
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            if not isinstance(data, dict):
                raise TypeError()
            componentsschemas_eyes_slot_type_1 = SlotError.from_dict(data)

            return componentsschemas_eyes_slot_type_1

        eyes = _parse_eyes(d.pop("eyes"))

        gripper = GripperSlots.from_dict(d.pop("gripper"))

        hand = SnapshotHand.from_dict(d.pop("hand"))

        kill = KillSnapshot.from_dict(d.pop("kill"))

        def _parse_neck(data: object) -> NeckState | SlotError:
            try:
                if not isinstance(data, dict):
                    raise TypeError()
                componentsschemas_neck_slot_type_0 = NeckState.from_dict(data)

                return componentsschemas_neck_slot_type_0
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            if not isinstance(data, dict):
                raise TypeError()
            componentsschemas_neck_slot_type_1 = SlotError.from_dict(data)

            return componentsschemas_neck_slot_type_1

        neck = _parse_neck(d.pop("neck"))

        def _parse_slider(data: object) -> SliderState | SlotError:
            try:
                if not isinstance(data, dict):
                    raise TypeError()
                componentsschemas_slider_slot_type_0 = SliderState.from_dict(data)

                return componentsschemas_slider_slot_type_0
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            if not isinstance(data, dict):
                raise TypeError()
            componentsschemas_slider_slot_type_1 = SlotError.from_dict(data)

            return componentsschemas_slider_slot_type_1

        slider = _parse_slider(d.pop("slider"))

        snapshot = cls(
            arm=arm,
            chassis=chassis,
            eyes=eyes,
            gripper=gripper,
            hand=hand,
            kill=kill,
            neck=neck,
            slider=slider,
        )

        snapshot.additional_properties = d
        return snapshot

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
