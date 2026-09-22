from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

T = TypeVar("T", bound="EyesPlaybackState")


@_attrs_define
class EyesPlaybackState:
    """Live state for a named eye-expression playback.

    Attributes:
        elapsed_ms (int): Wall-clock milliseconds elapsed since playback started.
        name (str): Catalog entry name resolved for the playback.
        total_ms (int): Total wall-clock playback duration in milliseconds.
    """

    elapsed_ms: int
    name: str
    total_ms: int
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        elapsed_ms = self.elapsed_ms

        name = self.name

        total_ms = self.total_ms

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "elapsed_ms": elapsed_ms,
                "name": name,
                "total_ms": total_ms,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        elapsed_ms = d.pop("elapsed_ms")

        name = d.pop("name")

        total_ms = d.pop("total_ms")

        eyes_playback_state = cls(
            elapsed_ms=elapsed_ms,
            name=name,
            total_ms=total_ms,
        )

        eyes_playback_state.additional_properties = d
        return eyes_playback_state

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
