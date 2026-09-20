from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

T = TypeVar("T", bound="NetInterface")


@_attrs_define
class NetInterface:
    """One network interface and its addresses (from `ip addr`).

    Attributes:
        addrs (list[str]): Assigned addresses in CIDR form (both IPv4 and IPv6).
        name (str): Interface name (e.g. `br0`).
        state (str): Operational state token as reported by the kernel (e.g. `UP`, `DOWN`,
            `UNKNOWN`).
    """

    addrs: list[str]
    name: str
    state: str
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        addrs = self.addrs

        name = self.name

        state = self.state

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "addrs": addrs,
                "name": name,
                "state": state,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        addrs = cast(list[str], d.pop("addrs"))

        name = d.pop("name")

        state = d.pop("state")

        net_interface = cls(
            addrs=addrs,
            name=name,
            state=state,
        )

        net_interface.additional_properties = d
        return net_interface

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
