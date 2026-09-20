from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

T = TypeVar("T", bound="KernelLog")


@_attrs_define
class KernelLog:
    """A verbatim tail of the board's kernel ring buffer (`dmesg`).

    Attributes:
        captured_unix_ms (int): Capture time on the firmware host, Unix epoch milliseconds.
        lines (list[str]): The kernel log lines, verbatim and in order.
        source (str): Where this report came from (e.g. `root@192.168.9.100` or `sim`).
    """

    captured_unix_ms: int
    lines: list[str]
    source: str
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        captured_unix_ms = self.captured_unix_ms

        lines = self.lines

        source = self.source

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "captured_unix_ms": captured_unix_ms,
                "lines": lines,
                "source": source,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        captured_unix_ms = d.pop("captured_unix_ms")

        lines = cast(list[str], d.pop("lines"))

        source = d.pop("source")

        kernel_log = cls(
            captured_unix_ms=captured_unix_ms,
            lines=lines,
            source=source,
        )

        kernel_log.additional_properties = d
        return kernel_log

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
