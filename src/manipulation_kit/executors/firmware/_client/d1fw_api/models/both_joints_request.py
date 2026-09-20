from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

T = TypeVar("T", bound="BothJointsRequest")


@_attrs_define
class BothJointsRequest:
    """`POST /v1/arm/move_joints_both`.

    Attributes:
        a (list[float]): Arm A target joint angles in degrees, seven joints.
        b (list[float]): Arm B target joint angles in degrees, seven joints.
        wait (bool): Block until both arms report arrival (up to 30 s).
    """

    a: list[float]
    b: list[float]
    wait: bool
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        a = self.a

        b = self.b

        wait = self.wait

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "a": a,
                "b": b,
                "wait": wait,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        a = cast(list[float], d.pop("a"))

        b = cast(list[float], d.pop("b"))

        wait = d.pop("wait")

        both_joints_request = cls(
            a=a,
            b=b,
            wait=wait,
        )

        both_joints_request.additional_properties = d
        return both_joints_request

    @property
    def additional_keys(self) -> list[str]:
        return list(self.additional_properties.keys())

    def __getitem__(self, key: str) -> Any:
        return self.additional_properties[key]

    def __setitem__(self, key: str, value: Any) -> None:
        self.additional_properties[key] = value

    def __delitem__(self, key: str) -> None:
        del self.additional_properties[key]

    def __contains__(self, key: str) -> bool:
        return key in self.additional_properties
