from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar

from attrs import define as _attrs_define
from typing_extensions import Self

if TYPE_CHECKING:
    from ..models.chassis_plan_draft import ChassisPlanDraft


T = TypeVar("T", bound="ChassisPlansCreate")


@_attrs_define
class ChassisPlansCreate:
    """Append one inactive timed plan after fresh collection comparison.

    Attributes:
        draft (ChassisPlanDraft): Timed draft. Cyclic interval units are unverified and unsupported.
        expected_revision (str): Opaque whole-collection token from read.
        scene (str): Existing normal scene.
    """

    draft: ChassisPlanDraft
    expected_revision: str
    scene: str

    def to_dict(self) -> dict[str, Any]:
        draft = self.draft.to_dict()

        expected_revision = self.expected_revision

        scene = self.scene

        field_dict: dict[str, Any] = {}

        field_dict.update(
            {
                "draft": draft,
                "expected_revision": expected_revision,
                "scene": scene,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.chassis_plan_draft import ChassisPlanDraft

        d = dict(src_dict)
        draft = ChassisPlanDraft.from_dict(d.pop("draft"))

        expected_revision = d.pop("expected_revision")

        scene = d.pop("scene")

        chassis_plans_create = cls(
            draft=draft,
            expected_revision=expected_revision,
            scene=scene,
        )

        return chassis_plans_create
