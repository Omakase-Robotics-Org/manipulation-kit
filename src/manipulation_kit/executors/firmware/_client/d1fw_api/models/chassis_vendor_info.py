from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..types import UNSET, Unset

T = TypeVar("T", bound="ChassisVendorInfo")


@_attrs_define
class ChassisVendorInfo:
    """The mobile base's vendor-reported identity and firmware/software build
    strings, as returned by the chassis controller's `getRobotInfo` endpoint.

    These fields are informational only (nothing in this core acts on them);
    they exist so that a vendor identity or firmware change on the chassis PC
    — otherwise silent, since the daemon only ever polls state — is observable
    through the same state and telemetry surfaces as any other chassis
    reading.

        Attributes:
            generation (str): Which generation of the vendor's mobile-base firmware the identity and
                version strings below were decided to belong to: `gs-2026`,
                `corona-2026`, or `unknown` for an identity this firmware version
                cannot place (including a base that has not answered yet).

                Unlike the five raw strings, this one the daemon DOES act on: it
                decides which source the battery reading comes from, and whether a
                speed command or a parameter write can be expected to have any effect.
            auto_version (None | str | Unset): Vendor `autoVersion`, the autonomy-navigation firmware build.
            master_version (None | str | Unset): Vendor `masterVersion`, the chassis controller's firmware build.
            robot_no (None | str | Unset): Vendor `robotNo`, the chassis PC's own identity string.
            single_chip_version (None | str | Unset): Vendor `singleChipVersion`, the single-chip (MCU) firmware build.
            web_version (None | str | Unset): Vendor `webVersion`, the chassis PC's web-application build.
    """

    generation: str
    auto_version: None | str | Unset = UNSET
    master_version: None | str | Unset = UNSET
    robot_no: None | str | Unset = UNSET
    single_chip_version: None | str | Unset = UNSET
    web_version: None | str | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        generation = self.generation

        auto_version: None | str | Unset
        if isinstance(self.auto_version, Unset):
            auto_version = UNSET
        else:
            auto_version = self.auto_version

        master_version: None | str | Unset
        if isinstance(self.master_version, Unset):
            master_version = UNSET
        else:
            master_version = self.master_version

        robot_no: None | str | Unset
        if isinstance(self.robot_no, Unset):
            robot_no = UNSET
        else:
            robot_no = self.robot_no

        single_chip_version: None | str | Unset
        if isinstance(self.single_chip_version, Unset):
            single_chip_version = UNSET
        else:
            single_chip_version = self.single_chip_version

        web_version: None | str | Unset
        if isinstance(self.web_version, Unset):
            web_version = UNSET
        else:
            web_version = self.web_version

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "generation": generation,
            }
        )
        if auto_version is not UNSET:
            field_dict["auto_version"] = auto_version
        if master_version is not UNSET:
            field_dict["master_version"] = master_version
        if robot_no is not UNSET:
            field_dict["robot_no"] = robot_no
        if single_chip_version is not UNSET:
            field_dict["single_chip_version"] = single_chip_version
        if web_version is not UNSET:
            field_dict["web_version"] = web_version

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        generation = d.pop("generation")

        def _parse_auto_version(data: object) -> None | str | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(None | str | Unset, data)

        auto_version = _parse_auto_version(d.pop("auto_version", UNSET))

        def _parse_master_version(data: object) -> None | str | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(None | str | Unset, data)

        master_version = _parse_master_version(d.pop("master_version", UNSET))

        def _parse_robot_no(data: object) -> None | str | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(None | str | Unset, data)

        robot_no = _parse_robot_no(d.pop("robot_no", UNSET))

        def _parse_single_chip_version(data: object) -> None | str | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(None | str | Unset, data)

        single_chip_version = _parse_single_chip_version(
            d.pop("single_chip_version", UNSET)
        )

        def _parse_web_version(data: object) -> None | str | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(None | str | Unset, data)

        web_version = _parse_web_version(d.pop("web_version", UNSET))

        chassis_vendor_info = cls(
            generation=generation,
            auto_version=auto_version,
            master_version=master_version,
            robot_no=robot_no,
            single_chip_version=single_chip_version,
            web_version=web_version,
        )

        chassis_vendor_info.additional_properties = d
        return chassis_vendor_info

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
