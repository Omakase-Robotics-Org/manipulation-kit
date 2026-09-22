from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from typing_extensions import Self

T = TypeVar("T", bound="MotorsRequest")


@_attrs_define
class MotorsRequest:
    """Requested drive-motor engagement.

    Attributes:
        engaged (bool): Engage powered drive when true; release for manual pushing when false.
    """

    engaged: bool

    def to_dict(self) -> dict[str, Any]:
        engaged = self.engaged

        field_dict: dict[str, Any] = {}

        field_dict.update(
            {
                "engaged": engaged,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        engaged = d.pop("engaged")

        motors_request = cls(
            engaged=engaged,
        )

        return motors_request
