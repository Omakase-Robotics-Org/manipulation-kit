from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.grip_preset import GripPreset
from ..types import UNSET, Unset

T = TypeVar("T", bound="GripperTarget")


@_attrs_define
class GripperTarget:
    """A continuous gripper target.

    Exactly one of `closedness` and `jaw_rad` is given. `closedness` is the hand-protocol
    convention shared with dx-manipulator's wire units: `0.0` is the commanded
    open ceiling, `1.0` the closed stop, linear in motor radians between them.
    `jaw_rad` addresses the motor directly, in the same radians the stroke
    report's `jaw_rad` uses. The backend resolves either to a motor target and
    refuses values outside the mechanism's range. The flattened
    [`GripRequest`] fields (`grip`, `grip_preload_rad`) say how hard to hold
    whatever the stroke meets on the way.

        Attributes:
            grip (GripPreset | None | Unset):
            grip_preload_rad (float | None | Unset): An explicit standing preload, in motor radians.
            closedness (float | None | Unset): Fraction of the stroke from open (0) to closed (1).
            jaw_rad (float | None | Unset): Absolute jaw position in motor radians.
    """

    grip: GripPreset | None | Unset = UNSET
    grip_preload_rad: float | None | Unset = UNSET
    closedness: float | None | Unset = UNSET
    jaw_rad: float | None | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        grip: None | str | Unset
        if isinstance(self.grip, Unset):
            grip = UNSET
        elif isinstance(self.grip, GripPreset):
            grip = self.grip.value
        else:
            grip = self.grip

        grip_preload_rad: float | None | Unset
        if isinstance(self.grip_preload_rad, Unset):
            grip_preload_rad = UNSET
        else:
            grip_preload_rad = self.grip_preload_rad

        closedness: float | None | Unset
        if isinstance(self.closedness, Unset):
            closedness = UNSET
        else:
            closedness = self.closedness

        jaw_rad: float | None | Unset
        if isinstance(self.jaw_rad, Unset):
            jaw_rad = UNSET
        else:
            jaw_rad = self.jaw_rad

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update({})
        if grip is not UNSET:
            field_dict["grip"] = grip
        if grip_preload_rad is not UNSET:
            field_dict["grip_preload_rad"] = grip_preload_rad
        if closedness is not UNSET:
            field_dict["closedness"] = closedness
        if jaw_rad is not UNSET:
            field_dict["jaw_rad"] = jaw_rad

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)

        def _parse_grip(data: object) -> GripPreset | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            try:
                if not isinstance(data, str):
                    raise TypeError()
                grip_type_1 = GripPreset(data)

                return grip_type_1
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            return cast(GripPreset | None | Unset, data)

        grip = _parse_grip(d.pop("grip", UNSET))

        def _parse_grip_preload_rad(data: object) -> float | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(float | None | Unset, data)

        grip_preload_rad = _parse_grip_preload_rad(d.pop("grip_preload_rad", UNSET))

        def _parse_closedness(data: object) -> float | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(float | None | Unset, data)

        closedness = _parse_closedness(d.pop("closedness", UNSET))

        def _parse_jaw_rad(data: object) -> float | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(float | None | Unset, data)

        jaw_rad = _parse_jaw_rad(d.pop("jaw_rad", UNSET))

        gripper_target = cls(
            grip=grip,
            grip_preload_rad=grip_preload_rad,
            closedness=closedness,
            jaw_rad=jaw_rad,
        )

        gripper_target.additional_properties = d
        return gripper_target

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
