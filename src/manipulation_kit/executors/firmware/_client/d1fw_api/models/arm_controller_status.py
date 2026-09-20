from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.controller_health import ControllerHealth

if TYPE_CHECKING:
    from ..models.arm_controller_identity import ArmControllerIdentity
    from ..models.kernel_module import KernelModule
    from ..models.net_interface import NetInterface
    from ..models.vendor_process import VendorProcess


T = TypeVar("T", bound="ArmControllerStatus")


@_attrs_define
class ArmControllerStatus:
    """Board device state: identity, vendor processes, kernel modules, network,
    and a derived health verdict.

        Attributes:
            captured_unix_ms (int): Capture time on the firmware host, Unix epoch milliseconds.
            health (ControllerHealth): Derived health of the arm controller.

                Health is DERIVED from the structured anatomy (process/module presence,
                slave AL states, recent fault severity) — never from mere output stability.
            identity (ArmControllerIdentity): Identity of the arm controller board, read from `hostname`, `uname`,
                `/etc/os-release`, `uptime`, and `date`.
            modules (list[KernelModule]): Loaded kernel modules of interest.
            network (list[NetInterface]): Network interfaces and their addresses.
            processes (list[VendorProcess]): Vendor processes seen running.
            reachable (bool): Whether the board answered the diagnostics request.
            source (str): Where this report came from (e.g. `root@192.168.9.100` or `sim`).
    """

    captured_unix_ms: int
    health: ControllerHealth
    identity: ArmControllerIdentity
    modules: list[KernelModule]
    network: list[NetInterface]
    processes: list[VendorProcess]
    reachable: bool
    source: str
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        captured_unix_ms = self.captured_unix_ms

        health = self.health.value

        identity = self.identity.to_dict()

        modules = []
        for modules_item_data in self.modules:
            modules_item = modules_item_data.to_dict()
            modules.append(modules_item)

        network = []
        for network_item_data in self.network:
            network_item = network_item_data.to_dict()
            network.append(network_item)

        processes = []
        for processes_item_data in self.processes:
            processes_item = processes_item_data.to_dict()
            processes.append(processes_item)

        reachable = self.reachable

        source = self.source

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "captured_unix_ms": captured_unix_ms,
                "health": health,
                "identity": identity,
                "modules": modules,
                "network": network,
                "processes": processes,
                "reachable": reachable,
                "source": source,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.arm_controller_identity import (
            ArmControllerIdentity,
        )
        from ..models.kernel_module import KernelModule
        from ..models.net_interface import NetInterface
        from ..models.vendor_process import VendorProcess

        d = dict(src_dict)
        captured_unix_ms = d.pop("captured_unix_ms")

        health = ControllerHealth(d.pop("health"))

        identity = ArmControllerIdentity.from_dict(d.pop("identity"))

        modules = []
        _modules = d.pop("modules")
        for modules_item_data in _modules:
            modules_item = KernelModule.from_dict(modules_item_data)

            modules.append(modules_item)

        network = []
        _network = d.pop("network")
        for network_item_data in _network:
            network_item = NetInterface.from_dict(network_item_data)

            network.append(network_item)

        processes = []
        _processes = d.pop("processes")
        for processes_item_data in _processes:
            processes_item = VendorProcess.from_dict(processes_item_data)

            processes.append(processes_item)

        reachable = d.pop("reachable")

        source = d.pop("source")

        arm_controller_status = cls(
            captured_unix_ms=captured_unix_ms,
            health=health,
            identity=identity,
            modules=modules,
            network=network,
            processes=processes,
            reachable=reachable,
            source=source,
        )

        arm_controller_status.additional_properties = d
        return arm_controller_status

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
