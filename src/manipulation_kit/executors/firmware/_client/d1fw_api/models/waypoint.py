from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

T = TypeVar("T", bound="Waypoint")


@_attrs_define
class Waypoint:
    """Timestamped dual-arm sample (absolute seconds from trajectory start).

    Attributes:
        a (list[float]): Physical left arm, seven joint angles in degrees.
        b (list[float]): Physical right arm, seven joint angles in degrees.
        t (float): Seconds, starting at zero, strictly increasing.
    """

    a: list[float]
    b: list[float]
    t: float
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        a = self.a

        b = self.b

        t = self.t

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "a": a,
                "b": b,
                "t": t,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        a = cast(list[float], d.pop("a"))

        b = cast(list[float], d.pop("b"))

        t = d.pop("t")

        waypoint = cls(
            a=a,
            b=b,
            t=t,
        )

        waypoint.additional_properties = d
        return waypoint

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
