from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from typing_extensions import Self

T = TypeVar("T", bound="ChassisPlanTarget")


@_attrs_define
class ChassisPlanTarget:
    """Exact identity and content selected from the previous raw snapshot.

    Attributes:
        expected_plan (Any): Entire original raw plan, including all fields.
        plan_id (str): Positional vendor ID, invalidated by collection changes.
    """

    expected_plan: Any
    plan_id: str

    def to_dict(self) -> dict[str, Any]:
        expected_plan = self.expected_plan

        plan_id = self.plan_id

        field_dict: dict[str, Any] = {}

        field_dict.update(
            {
                "expected_plan": expected_plan,
                "plan_id": plan_id,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        expected_plan = d.pop("expected_plan")

        plan_id = d.pop("plan_id")

        chassis_plan_target = cls(
            expected_plan=expected_plan,
            plan_id=plan_id,
        )

        return chassis_plan_target
