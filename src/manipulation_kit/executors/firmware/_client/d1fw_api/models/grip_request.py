from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.grip_preset import GripPreset
from ..types import UNSET, Unset

T = TypeVar("T", bound="GripRequest")


@_attrs_define
class GripRequest:
    """How hard a closing stroke squeezes and holds what it meets.

    At most one of the two fields is given: a named [`GripPreset`], or an
    explicit standing preload in motor radians (which keeps the `firm`
    preset's stop torque). Neither means the backend's configured default,
    `firm`. Carried by [`GripperTarget`] and by a bare `close`.

        Attributes:
            grip (GripPreset | None | Unset):
            grip_preload_rad (float | None | Unset): An explicit standing preload, in motor radians.
    """

    grip: GripPreset | None | Unset = UNSET
    grip_preload_rad: float | None | Unset = UNSET
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

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update({})
        if grip is not UNSET:
            field_dict["grip"] = grip
        if grip_preload_rad is not UNSET:
            field_dict["grip_preload_rad"] = grip_preload_rad

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

        grip_request = cls(
            grip=grip,
            grip_preload_rad=grip_preload_rad,
        )

        grip_request.additional_properties = d
        return grip_request

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
