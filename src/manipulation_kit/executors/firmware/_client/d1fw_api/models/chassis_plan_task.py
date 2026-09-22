from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from typing_extensions import Self

T = TypeVar("T", bound="ChassisPlanTask")


@_attrs_define
class ChassisPlanTask:
    """A manufacturer task row, excluding server-assigned positional taskID.

    Attributes:
        extra_parm (str): Manufacturer extra parameter; blank normalizes to three hyphens.
        goal_id (str): Exact point ID from the selected scene's fresh point list.
        task_type (str): Manufacturer type: 0 walking, 1 disinfection, 2 voice; effects unverified.
        work_time (str): Manufacturer workTime string; no unit is inferred.
    """

    extra_parm: str
    goal_id: str
    task_type: str
    work_time: str

    def to_dict(self) -> dict[str, Any]:
        extra_parm = self.extra_parm

        goal_id = self.goal_id

        task_type = self.task_type

        work_time = self.work_time

        field_dict: dict[str, Any] = {}

        field_dict.update(
            {
                "extra_parm": extra_parm,
                "goal_id": goal_id,
                "task_type": task_type,
                "work_time": work_time,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        extra_parm = d.pop("extra_parm")

        goal_id = d.pop("goal_id")

        task_type = d.pop("task_type")

        work_time = d.pop("work_time")

        chassis_plan_task = cls(
            extra_parm=extra_parm,
            goal_id=goal_id,
            task_type=task_type,
            work_time=work_time,
        )

        return chassis_plan_task
