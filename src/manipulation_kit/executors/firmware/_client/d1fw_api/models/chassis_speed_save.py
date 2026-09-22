from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from typing_extensions import Self

from ..models.chassis_speed_profile import ChassisSpeedProfile

T = TypeVar("T", bound="ChassisSpeedSave")


@_attrs_define
class ChassisSpeedSave:
    """Conditional save of one profile through setSpeedSetting, not setParams.

    Attributes:
        expected (Any): Complete original getSpeedSetting object, including unknown fields.
        profile (ChassisSpeedProfile): Exact manufacturer saved-profile selectors; narrow has no verified writer.
        value (float): Finite positive saved proposal. No verified vendor upper bound is claimed.
    """

    expected: Any
    profile: ChassisSpeedProfile
    value: float

    def to_dict(self) -> dict[str, Any]:
        expected = self.expected

        profile = self.profile.value

        value = self.value

        field_dict: dict[str, Any] = {}

        field_dict.update(
            {
                "expected": expected,
                "profile": profile,
                "value": value,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        expected = d.pop("expected")

        profile = ChassisSpeedProfile(d.pop("profile"))

        value = d.pop("value")

        chassis_speed_save = cls(
            expected=expected,
            profile=profile,
            value=value,
        )

        return chassis_speed_save
