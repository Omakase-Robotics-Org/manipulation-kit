from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

T = TypeVar("T", bound="ArmControllerIdentity")


@_attrs_define
class ArmControllerIdentity:
    """Identity of the arm controller board, read from `hostname`, `uname`,
    `/etc/os-release`, `uptime`, and `date`.

        Attributes:
            controller_time (str): The board's own wall-clock reading from `date`, preserved verbatim
                (the board reports it in its own locale).
            hostname (str): Reported host name (e.g. `FUSION`).
            kernel (str): Full `uname -a` kernel string.
            os_release (str): Human-readable OS release (the `PRETTY_NAME` from `/etc/os-release`).
            uptime (str): Raw `uptime` line, including load averages.
    """

    controller_time: str
    hostname: str
    kernel: str
    os_release: str
    uptime: str
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        controller_time = self.controller_time

        hostname = self.hostname

        kernel = self.kernel

        os_release = self.os_release

        uptime = self.uptime

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "controller_time": controller_time,
                "hostname": hostname,
                "kernel": kernel,
                "os_release": os_release,
                "uptime": uptime,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        controller_time = d.pop("controller_time")

        hostname = d.pop("hostname")

        kernel = d.pop("kernel")

        os_release = d.pop("os_release")

        uptime = d.pop("uptime")

        arm_controller_identity = cls(
            controller_time=controller_time,
            hostname=hostname,
            kernel=kernel,
            os_release=os_release,
            uptime=uptime,
        )

        arm_controller_identity.additional_properties = d
        return arm_controller_identity

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
