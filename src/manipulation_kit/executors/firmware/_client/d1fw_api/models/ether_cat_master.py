from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

if TYPE_CHECKING:
    from ..models.ether_cat_slave import EtherCatSlave


T = TypeVar("T", bound="EtherCatMaster")


@_attrs_define
class EtherCatMaster:
    """One EtherCAT master and its ring (from `ethercat master` + `ethercat
    slaves`).

        Attributes:
            active (bool): Whether the master is active.
            index (int): Master index (e.g. `0` for `Master0`).
            link_up (bool): Whether the main Ethernet link is up.
            lost_frames (int): Cumulative lost frame count.
            phase (str): Operational phase string (e.g. `Operation`).
            slave_count (int): Number of slaves the master reports on its ring.
            slaves (list[EtherCatSlave]): The slaves attached to this master, in ring order.
    """

    active: bool
    index: int
    link_up: bool
    lost_frames: int
    phase: str
    slave_count: int
    slaves: list[EtherCatSlave]
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        active = self.active

        index = self.index

        link_up = self.link_up

        lost_frames = self.lost_frames

        phase = self.phase

        slave_count = self.slave_count

        slaves = []
        for slaves_item_data in self.slaves:
            slaves_item = slaves_item_data.to_dict()
            slaves.append(slaves_item)

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "active": active,
                "index": index,
                "link_up": link_up,
                "lost_frames": lost_frames,
                "phase": phase,
                "slave_count": slave_count,
                "slaves": slaves,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.ether_cat_slave import EtherCatSlave

        d = dict(src_dict)
        active = d.pop("active")

        index = d.pop("index")

        link_up = d.pop("link_up")

        lost_frames = d.pop("lost_frames")

        phase = d.pop("phase")

        slave_count = d.pop("slave_count")

        slaves = []
        _slaves = d.pop("slaves")
        for slaves_item_data in _slaves:
            slaves_item = EtherCatSlave.from_dict(slaves_item_data)

            slaves.append(slaves_item)

        ether_cat_master = cls(
            active=active,
            index=index,
            link_up=link_up,
            lost_frames=lost_frames,
            phase=phase,
            slave_count=slave_count,
            slaves=slaves,
        )

        ether_cat_master.additional_properties = d
        return ether_cat_master

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
