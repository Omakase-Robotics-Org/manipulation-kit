from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..types import UNSET, Unset

T = TypeVar("T", bound="EyesExpressionRequest")


@_attrs_define
class EyesExpressionRequest:
    """`POST /v1/eyes/set_expression`.

    Attributes:
        name (str): Catalog expression name, as listed by `GET /v1/eyes/list_expressions`.
        loops (int | None | Unset): Loop count; omitted means `0`, which defers to the document's own
            loop policy.  The service ceiling is 20 loops.
        speed (float | None | Unset): Playback speed multiplier; omitted means `1.0`.
    """

    name: str
    loops: int | None | Unset = UNSET
    speed: float | None | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        name = self.name

        loops: int | None | Unset
        if isinstance(self.loops, Unset):
            loops = UNSET
        else:
            loops = self.loops

        speed: float | None | Unset
        if isinstance(self.speed, Unset):
            speed = UNSET
        else:
            speed = self.speed

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "name": name,
            }
        )
        if loops is not UNSET:
            field_dict["loops"] = loops
        if speed is not UNSET:
            field_dict["speed"] = speed

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        name = d.pop("name")

        def _parse_loops(data: object) -> int | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(int | None | Unset, data)

        loops = _parse_loops(d.pop("loops", UNSET))

        def _parse_speed(data: object) -> float | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(float | None | Unset, data)

        speed = _parse_speed(d.pop("speed", UNSET))

        eyes_expression_request = cls(
            name=name,
            loops=loops,
            speed=speed,
        )

        eyes_expression_request.additional_properties = d
        return eyes_expression_request

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
