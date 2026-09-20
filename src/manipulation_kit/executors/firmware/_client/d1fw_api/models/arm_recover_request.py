from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from typing_extensions import Self

from ..types import UNSET, Unset

T = TypeVar("T", bound="ArmRecoverRequest")


@_attrs_define
class ArmRecoverRequest:
    """Explicit recovery command; ratios are normalized, not percentages.

    Attributes:
        acc_ratio (float | Unset): Position-mode acceleration ratio (default 0.05). Default: 0.05000000074505806.
        vel_ratio (float | Unset): Position-mode velocity ratio (default 0.05). Default: 0.05000000074505806.
    """

    acc_ratio: float | Unset = 0.05000000074505806
    vel_ratio: float | Unset = 0.05000000074505806

    def to_dict(self) -> dict[str, Any]:
        acc_ratio = self.acc_ratio

        vel_ratio = self.vel_ratio

        field_dict: dict[str, Any] = {}

        field_dict.update({})
        if acc_ratio is not UNSET:
            field_dict["acc_ratio"] = acc_ratio
        if vel_ratio is not UNSET:
            field_dict["vel_ratio"] = vel_ratio

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        acc_ratio = d.pop("acc_ratio", UNSET)

        vel_ratio = d.pop("vel_ratio", UNSET)

        arm_recover_request = cls(
            acc_ratio=acc_ratio,
            vel_ratio=vel_ratio,
        )

        return arm_recover_request
