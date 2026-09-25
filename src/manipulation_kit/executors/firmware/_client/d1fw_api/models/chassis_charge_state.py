from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.charge_basis import ChargeBasis
from ..models.charge_dock_read_status import ChargeDockReadStatus
from ..models.charge_dock_source import ChargeDockSource
from ..models.dock_phase import DockPhase
from ..models.pack_charge import PackCharge
from ..types import UNSET, Unset

if TYPE_CHECKING:
    from ..models.charge_dock import ChargeDock


T = TypeVar("T", bound="ChassisChargeState")


@_attrs_define
class ChassisChargeState:
    """Everything the daemon knows about the mobile base's charging, in one
    object: where the base is in its docking routine, whether automatic
    charging is the mode it is in, what its battery packs say about current
    flow, and which waypoint pressing "charge" would send it to.

    The four are separate fields because they are four different measurements
    that routinely disagree, and folding them into one flag is what makes an
    operator guess. The vendor's own sequence, from its example program
    `7.add_charge.py`, is: start automatic charging, and `workMode` goes 1 to 2
    while `dockStatus` goes 0 to 1; docking completes and `dockStatus` becomes
    2; the base leaves and `dockStatus` goes 3 then 0 while `workMode` returns
    to 1. Through all of that, whether current is actually flowing is reported
    only by the battery pack.

        Attributes:
            auto_charging (bool): Whether the base's work mode is the vendor's automatic-charging mode
                (`workMode == 2`).

                This is the base's MODE, not a measurement of current: a base can sit
                in `workMode` 2 having failed to reach its dock.
            basis (ChargeBasis): Where the daemon's `charging` answer came from, or that it has no source.

                The two mobile-base firmware generations report charge over different
                transports and only one of them is right for a given base, so the evidence
                travels with the answer rather than being left for a consumer to guess.
            dock (DockPhase): The automatic-charging (docking) phase the mobile base reports.

                This is the daemon's typed reading of the vendor's `dockStatus` field. It
                says where the base is in its docking routine and says NOTHING about
                whether current is flowing — only the battery pack reports that, which is
                why [`ChassisChargeState`] carries the two side by side.
            dock_point_source (ChargeDockSource): Where [`ChassisChargeState::dock_point`] came from.
            pack (PackCharge): What the mobile base's battery packs say about current flow, as a word.

                The tri-state `charging` boolean answers "is it charging"; this answers
                "what did the packs actually report", which is what tells an unreadable
                pack apart from a pack reporting a code this firmware has no name for.
            charging (bool | None | Unset): Whether the base is charging: `true` only on positive evidence,
                `false` only on positive evidence, and `null` when the only evidence
                is a code nothing documents or when there is no evidence at all.

                An undocumented pack code is NEVER folded into `false`. A consumer
                that must show something for `null` should show "unknown", not "not
                charging"; [`Self::basis`] says why it is `null`.
            dock_point (ChargeDock | None | Unset):
            dock_raw (None | str | Unset): The vendor's `dockStatus` string exactly as it arrived, or `null` when
                the base reported no such field.
            dock_read_status (ChargeDockReadStatus | Unset): Outcome of the latest vendor charging-dock read, independent of
                fallback provenance.
            pack_raw (int | None | Unset): The raw `charge_status` integer that decided [`Self::pack`], or `null`
                when no pack reported.
    """

    auto_charging: bool
    basis: ChargeBasis
    dock: DockPhase
    dock_point_source: ChargeDockSource
    pack: PackCharge
    charging: bool | None | Unset = UNSET
    dock_point: ChargeDock | None | Unset = UNSET
    dock_raw: None | str | Unset = UNSET
    dock_read_status: ChargeDockReadStatus | Unset = UNSET
    pack_raw: int | None | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        from ..models.charge_dock import ChargeDock

        auto_charging = self.auto_charging

        basis = self.basis.value

        dock = self.dock.value

        dock_point_source = self.dock_point_source.value

        pack = self.pack.value

        charging: bool | None | Unset
        if isinstance(self.charging, Unset):
            charging = UNSET
        else:
            charging = self.charging

        dock_point: dict[str, Any] | None | Unset
        if isinstance(self.dock_point, Unset):
            dock_point = UNSET
        elif isinstance(self.dock_point, ChargeDock):
            dock_point = self.dock_point.to_dict()
        else:
            dock_point = self.dock_point

        dock_raw: None | str | Unset
        if isinstance(self.dock_raw, Unset):
            dock_raw = UNSET
        else:
            dock_raw = self.dock_raw

        dock_read_status: str | Unset = UNSET
        if not isinstance(self.dock_read_status, Unset):
            dock_read_status = self.dock_read_status.value

        pack_raw: int | None | Unset
        if isinstance(self.pack_raw, Unset):
            pack_raw = UNSET
        else:
            pack_raw = self.pack_raw

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "auto_charging": auto_charging,
                "basis": basis,
                "dock": dock,
                "dock_point_source": dock_point_source,
                "pack": pack,
            }
        )
        if charging is not UNSET:
            field_dict["charging"] = charging
        if dock_point is not UNSET:
            field_dict["dock_point"] = dock_point
        if dock_raw is not UNSET:
            field_dict["dock_raw"] = dock_raw
        if dock_read_status is not UNSET:
            field_dict["dock_read_status"] = dock_read_status
        if pack_raw is not UNSET:
            field_dict["pack_raw"] = pack_raw

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.charge_dock import ChargeDock

        d = dict(src_dict)
        auto_charging = d.pop("auto_charging")

        basis = ChargeBasis(d.pop("basis"))

        dock = DockPhase(d.pop("dock"))

        dock_point_source = ChargeDockSource(d.pop("dock_point_source"))

        pack = PackCharge(d.pop("pack"))

        def _parse_charging(data: object) -> bool | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(bool | None | Unset, data)

        charging = _parse_charging(d.pop("charging", UNSET))

        def _parse_dock_point(data: object) -> ChargeDock | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            try:
                if not isinstance(data, dict):
                    raise TypeError()
                dock_point_type_1 = ChargeDock.from_dict(data)

                return dock_point_type_1
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            return cast(ChargeDock | None | Unset, data)

        dock_point = _parse_dock_point(d.pop("dock_point", UNSET))

        def _parse_dock_raw(data: object) -> None | str | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(None | str | Unset, data)

        dock_raw = _parse_dock_raw(d.pop("dock_raw", UNSET))

        _dock_read_status = d.pop("dock_read_status", UNSET)
        dock_read_status: ChargeDockReadStatus | Unset
        if isinstance(_dock_read_status, Unset):
            dock_read_status = UNSET
        else:
            dock_read_status = ChargeDockReadStatus(_dock_read_status)

        def _parse_pack_raw(data: object) -> int | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(int | None | Unset, data)

        pack_raw = _parse_pack_raw(d.pop("pack_raw", UNSET))

        chassis_charge_state = cls(
            auto_charging=auto_charging,
            basis=basis,
            dock=dock,
            dock_point_source=dock_point_source,
            pack=pack,
            charging=charging,
            dock_point=dock_point,
            dock_raw=dock_raw,
            dock_read_status=dock_read_status,
            pack_raw=pack_raw,
        )

        chassis_charge_state.additional_properties = d
        return chassis_charge_state

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
