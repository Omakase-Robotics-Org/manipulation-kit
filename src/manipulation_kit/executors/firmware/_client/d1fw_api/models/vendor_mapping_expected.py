from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from typing_extensions import Self

T = TypeVar("T", bound="VendorMappingExpected")


@_attrs_define
class VendorMappingExpected:
    """Operator-observed mode precondition, checked again immediately before sending.

    Attributes:
        auto_status (int): Vendor navigation/sensor status.
        current_scene (str): Current scene from the fresh monitor message.
        work_mode (int): Vendor decision work mode.
    """

    auto_status: int
    current_scene: str
    work_mode: int

    def to_dict(self) -> dict[str, Any]:
        auto_status = self.auto_status

        current_scene = self.current_scene

        work_mode = self.work_mode

        field_dict: dict[str, Any] = {}

        field_dict.update(
            {
                "auto_status": auto_status,
                "current_scene": current_scene,
                "work_mode": work_mode,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        auto_status = d.pop("auto_status")

        current_scene = d.pop("current_scene")

        work_mode = d.pop("work_mode")

        vendor_mapping_expected = cls(
            auto_status=auto_status,
            current_scene=current_scene,
            work_mode=work_mode,
        )

        return vendor_mapping_expected
