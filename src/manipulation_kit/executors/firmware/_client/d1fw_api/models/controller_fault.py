from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..types import UNSET, Unset

T = TypeVar("T", bound="ControllerFault")


@_attrs_define
class ControllerFault:
    """One controller fault scanned from the vendor `LOG.txt` tail.

    The originating log line is kept verbatim in `raw` because a fault log is
    inherently free-form; the other fields are the structured extraction.

        Attributes:
            codes (list[str]): Servo error codes carried by the line (e.g. `0xff51`), in order.
            raw (str): The original log line, verbatim.
            severity (str): Log severity token (e.g. `ERRO`, `WARN`).
            timestamp (str): The log line's own timestamp token (e.g. `00:15:38`).
            arm (None | str | Unset): The arm the fault names, if any (e.g. `arm0`).
    """

    codes: list[str]
    raw: str
    severity: str
    timestamp: str
    arm: None | str | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        codes = self.codes

        raw = self.raw

        severity = self.severity

        timestamp = self.timestamp

        arm: None | str | Unset
        if isinstance(self.arm, Unset):
            arm = UNSET
        else:
            arm = self.arm

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "codes": codes,
                "raw": raw,
                "severity": severity,
                "timestamp": timestamp,
            }
        )
        if arm is not UNSET:
            field_dict["arm"] = arm

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        codes = cast(list[str], d.pop("codes"))

        raw = d.pop("raw")

        severity = d.pop("severity")

        timestamp = d.pop("timestamp")

        def _parse_arm(data: object) -> None | str | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(None | str | Unset, data)

        arm = _parse_arm(d.pop("arm", UNSET))

        controller_fault = cls(
            codes=codes,
            raw=raw,
            severity=severity,
            timestamp=timestamp,
            arm=arm,
        )

        controller_fault.additional_properties = d
        return controller_fault

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
