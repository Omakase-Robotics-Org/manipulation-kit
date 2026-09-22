from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from typing_extensions import Self

T = TypeVar("T", bound="ChassisPlansRead")


@_attrs_define
class ChassisPlansRead:
    """One explicitly named normal scene; union and paths are forbidden.

    Attributes:
        scene (str): Existing normal scene name.
    """

    scene: str

    def to_dict(self) -> dict[str, Any]:
        scene = self.scene

        field_dict: dict[str, Any] = {}

        field_dict.update(
            {
                "scene": scene,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        scene = d.pop("scene")

        chassis_plans_read = cls(
            scene=scene,
        )

        return chassis_plans_read
