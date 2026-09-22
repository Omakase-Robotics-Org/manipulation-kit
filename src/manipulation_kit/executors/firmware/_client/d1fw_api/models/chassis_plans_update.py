from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar

from attrs import define as _attrs_define
from typing_extensions import Self

if TYPE_CHECKING:
    from ..models.chassis_plan_draft import ChassisPlanDraft
    from ..models.chassis_plan_target import ChassisPlanTarget


T = TypeVar("T", bound="ChassisPlansUpdate")


@_attrs_define
class ChassisPlansUpdate:
    """Replace a selected inactive plan after collection and target checks.

    Attributes:
        draft (ChassisPlanDraft): Timed draft. Cyclic interval units are unverified and unsupported.
        expected_revision (str): Opaque whole-collection token from read.
        scene (str): Existing normal scene.
        target (ChassisPlanTarget): Exact identity and content selected from the previous raw snapshot.
    """

    draft: ChassisPlanDraft
    expected_revision: str
    scene: str
    target: ChassisPlanTarget

    def to_dict(self) -> dict[str, Any]:
        draft = self.draft.to_dict()

        expected_revision = self.expected_revision

        scene = self.scene

        target = self.target.to_dict()

        field_dict: dict[str, Any] = {}

        field_dict.update(
            {
                "draft": draft,
                "expected_revision": expected_revision,
                "scene": scene,
                "target": target,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.chassis_plan_draft import ChassisPlanDraft
        from ..models.chassis_plan_target import ChassisPlanTarget

        d = dict(src_dict)
        draft = ChassisPlanDraft.from_dict(d.pop("draft"))

        expected_revision = d.pop("expected_revision")

        scene = d.pop("scene")

        target = ChassisPlanTarget.from_dict(d.pop("target"))

        chassis_plans_update = cls(
            draft=draft,
            expected_revision=expected_revision,
            scene=scene,
            target=target,
        )

        return chassis_plans_update
