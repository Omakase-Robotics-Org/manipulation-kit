from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.controller_health import ControllerHealth

if TYPE_CHECKING:
    from ..models.arm_calibration import ArmCalibration
    from ..models.controller_fault import ControllerFault
    from ..models.ether_cat_master import EtherCatMaster


T = TypeVar("T", bound="ArmControllerServos")


@_attrs_define
class ArmControllerServos:
    """Board servo/EtherCAT state: masters and their rings, per-joint calibration,
    recent faults, and a derived health verdict.

        Attributes:
            calibration (list[ArmCalibration]): Per-arm joint calibration from `robot.ini`.
            captured_unix_ms (int): Capture time on the firmware host, Unix epoch milliseconds.
            health (ControllerHealth): Derived health of the arm controller.

                Health is DERIVED from the structured anatomy (process/module presence,
                slave AL states, recent fault severity) — never from mere output stability.
            masters (list[EtherCatMaster]): EtherCAT masters and their slave rings.
            recent_faults (list[ControllerFault]): Recent controller faults scanned from the vendor log tail.
            source (str): Where this report came from (e.g. `root@192.168.9.100` or `sim`).
    """

    calibration: list[ArmCalibration]
    captured_unix_ms: int
    health: ControllerHealth
    masters: list[EtherCatMaster]
    recent_faults: list[ControllerFault]
    source: str
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        calibration = []
        for calibration_item_data in self.calibration:
            calibration_item = calibration_item_data.to_dict()
            calibration.append(calibration_item)

        captured_unix_ms = self.captured_unix_ms

        health = self.health.value

        masters = []
        for masters_item_data in self.masters:
            masters_item = masters_item_data.to_dict()
            masters.append(masters_item)

        recent_faults = []
        for recent_faults_item_data in self.recent_faults:
            recent_faults_item = recent_faults_item_data.to_dict()
            recent_faults.append(recent_faults_item)

        source = self.source

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "calibration": calibration,
                "captured_unix_ms": captured_unix_ms,
                "health": health,
                "masters": masters,
                "recent_faults": recent_faults,
                "source": source,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.arm_calibration import ArmCalibration
        from ..models.controller_fault import ControllerFault
        from ..models.ether_cat_master import EtherCatMaster

        d = dict(src_dict)
        calibration = []
        _calibration = d.pop("calibration")
        for calibration_item_data in _calibration:
            calibration_item = ArmCalibration.from_dict(calibration_item_data)

            calibration.append(calibration_item)

        captured_unix_ms = d.pop("captured_unix_ms")

        health = ControllerHealth(d.pop("health"))

        masters = []
        _masters = d.pop("masters")
        for masters_item_data in _masters:
            masters_item = EtherCatMaster.from_dict(masters_item_data)

            masters.append(masters_item)

        recent_faults = []
        _recent_faults = d.pop("recent_faults")
        for recent_faults_item_data in _recent_faults:
            recent_faults_item = ControllerFault.from_dict(recent_faults_item_data)

            recent_faults.append(recent_faults_item)

        source = d.pop("source")

        arm_controller_servos = cls(
            calibration=calibration,
            captured_unix_ms=captured_unix_ms,
            health=health,
            masters=masters,
            recent_faults=recent_faults,
            source=source,
        )

        arm_controller_servos.additional_properties = d
        return arm_controller_servos

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
