from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

T = TypeVar("T", bound="HealthStatus")


@_attrs_define
class HealthStatus:
    """`GET /v1/health`.

    Attributes:
        backend (str): The backend the daemon was started with, such as `real` or `sim`.
        uptime_s (int): Whole seconds since this frontend started serving.

            The WebSocket `health` method reports the same field with fractional
            seconds.
        version (str): The daemon's package version.
    """

    backend: str
    uptime_s: int
    version: str
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        backend = self.backend

        uptime_s = self.uptime_s

        version = self.version

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "backend": backend,
                "uptime_s": uptime_s,
                "version": version,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        backend = d.pop("backend")

        uptime_s = d.pop("uptime_s")

        version = d.pop("version")

        health_status = cls(
            backend=backend,
            uptime_s=uptime_s,
            version=version,
        )

        health_status.additional_properties = d
        return health_status

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
