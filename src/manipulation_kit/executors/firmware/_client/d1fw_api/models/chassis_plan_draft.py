from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar, cast

from attrs import define as _attrs_define
from typing_extensions import Self

if TYPE_CHECKING:
    from ..models.chassis_plan_task import ChassisPlanTask


T = TypeVar("T", bound="ChassisPlanDraft")


@_attrs_define
class ChassisPlanDraft:
    """Timed draft. Cyclic interval units are unverified and unsupported.

    Attributes:
        mode (str): Manufacturer mode 0, 1 or 2; not a physical capability claim.
        name (str): Display name.
        tasks (list[ChassisPlanTask]): Nonempty ordered point tasks.
        times (list[str]): Unique 24-hour HH:mm scheduled times in caller order.
        weekdays (list[int]): Unique weekdays: Monday 1 through Sunday 7.
    """

    mode: str
    name: str
    tasks: list[ChassisPlanTask]
    times: list[str]
    weekdays: list[int]

    def to_dict(self) -> dict[str, Any]:
        mode = self.mode

        name = self.name

        tasks = []
        for tasks_item_data in self.tasks:
            tasks_item = tasks_item_data.to_dict()
            tasks.append(tasks_item)

        times = self.times

        weekdays = self.weekdays

        field_dict: dict[str, Any] = {}

        field_dict.update(
            {
                "mode": mode,
                "name": name,
                "tasks": tasks,
                "times": times,
                "weekdays": weekdays,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.chassis_plan_task import ChassisPlanTask

        d = dict(src_dict)
        mode = d.pop("mode")

        name = d.pop("name")

        tasks = []
        _tasks = d.pop("tasks")
        for tasks_item_data in _tasks:
            tasks_item = ChassisPlanTask.from_dict(tasks_item_data)

            tasks.append(tasks_item)

        times = cast(list[str], d.pop("times"))

        weekdays = cast(list[int], d.pop("weekdays"))

        chassis_plan_draft = cls(
            mode=mode,
            name=name,
            tasks=tasks,
            times=times,
            weekdays=weekdays,
        )

        return chassis_plan_draft
