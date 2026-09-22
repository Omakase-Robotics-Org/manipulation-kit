from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.chassis_param_applies import ChassisParamApplies

T = TypeVar("T", bound="ChassisSetParamsReport")


@_attrs_define
class ChassisSetParamsReport:
    """What one [`ChassisParam`] write did.

    Three fields exist because a write can be accepted and still change
    nothing, in three independent ways: the older mobile-base firmware
    generation serves `setParams` and ignores what it is told
    ([`ChassisSetParamsReport::effective`]); the vendor writes a YAML file and
    no running node re-reads it, so the value waits for a restart, and for
    WHICH restart depends on the parameter
    ([`ChassisSetParamsReport::applies_when`]); and for one parameter no node
    reads the written file at all. All of it travels with the response rather
    than being left for the caller to know.

        Attributes:
            applies_when (ChassisParamApplies): When a written [`ChassisParam`] starts being used by the mobile base.

                Every `setParams` type writes a YAML file under the vendor's
                `<workspace>/install/share/robot_bringup/param/` and nothing more: no
                binary in the vendor's installed autonomy tree contains any of those file
                names, so no running node re-reads one. The written value therefore
                reaches the base only when the node whose launch file loads that YAML with
                `<rosparam command="load">` is started again — and WHICH node that is
                differs per type, which is why this is an enumeration and not a flag.

                [`ChassisParamApplies::NoKnownReader`] is not a restart at all: the write
                lands in a file that no launch file in the installed tree loads, so no
                restart makes it take effect.
            effective (bool): Whether this generation's firmware acts on this endpoint at all.
                `false` means the request was accepted and will do nothing.
            generation (str): The mobile-base firmware generation the write was sent to, as
                [`ChassisVendorInfo::generation`] spells it.
            kind (str): The parameter that was written, as [`ChassisParam::kind`] spells it.
            note (str): A one-line statement of the conditions above, for an operator reading
                the response by hand.
            value (str): The exact string sent as the vendor's `value`.
            vendor_file (str): The vendor YAML file the write lands in, relative to the chassis PC's
                `<workspace>/install/share/robot_bringup/param/`, or an empty string
                when it is not established. A parameter written into two files names
                both, separated by `", "`.
    """

    applies_when: ChassisParamApplies
    effective: bool
    generation: str
    kind: str
    note: str
    value: str
    vendor_file: str
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        applies_when = self.applies_when.value

        effective = self.effective

        generation = self.generation

        kind = self.kind

        note = self.note

        value = self.value

        vendor_file = self.vendor_file

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "applies_when": applies_when,
                "effective": effective,
                "generation": generation,
                "kind": kind,
                "note": note,
                "value": value,
                "vendor_file": vendor_file,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        applies_when = ChassisParamApplies(d.pop("applies_when"))

        effective = d.pop("effective")

        generation = d.pop("generation")

        kind = d.pop("kind")

        note = d.pop("note")

        value = d.pop("value")

        vendor_file = d.pop("vendor_file")

        chassis_set_params_report = cls(
            applies_when=applies_when,
            effective=effective,
            generation=generation,
            kind=kind,
            note=note,
            value=value,
            vendor_file=vendor_file,
        )

        chassis_set_params_report.additional_properties = d
        return chassis_set_params_report

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
