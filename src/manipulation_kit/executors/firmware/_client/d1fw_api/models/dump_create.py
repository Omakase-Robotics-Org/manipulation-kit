from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from typing_extensions import Self

from ..types import UNSET, Unset

T = TypeVar("T", bound="DumpCreate")


@_attrs_define
class DumpCreate:
    """`POST /v1/recorder/dumps`.

    Attributes:
        note (None | str | Unset): An operator's note to store with the dump.
        pinned (bool | None | Unset): Whether to pin it against retention immediately. Omitted means not
            pinned.
        post_trigger_s (int | None | Unset): Seconds to wait after the trigger before copying the ring. Omitted
            means `0`: an operator asking for a dump is asking about what has
            ALREADY happened, and a request that blocked for five seconds would
            look like a hung daemon.
    """

    note: None | str | Unset = UNSET
    pinned: bool | None | Unset = UNSET
    post_trigger_s: int | None | Unset = UNSET

    def to_dict(self) -> dict[str, Any]:
        note: None | str | Unset
        if isinstance(self.note, Unset):
            note = UNSET
        else:
            note = self.note

        pinned: bool | None | Unset
        if isinstance(self.pinned, Unset):
            pinned = UNSET
        else:
            pinned = self.pinned

        post_trigger_s: int | None | Unset
        if isinstance(self.post_trigger_s, Unset):
            post_trigger_s = UNSET
        else:
            post_trigger_s = self.post_trigger_s

        field_dict: dict[str, Any] = {}

        field_dict.update({})
        if note is not UNSET:
            field_dict["note"] = note
        if pinned is not UNSET:
            field_dict["pinned"] = pinned
        if post_trigger_s is not UNSET:
            field_dict["post_trigger_s"] = post_trigger_s

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)

        def _parse_note(data: object) -> None | str | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(None | str | Unset, data)

        note = _parse_note(d.pop("note", UNSET))

        def _parse_pinned(data: object) -> bool | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(bool | None | Unset, data)

        pinned = _parse_pinned(d.pop("pinned", UNSET))

        def _parse_post_trigger_s(data: object) -> int | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(int | None | Unset, data)

        post_trigger_s = _parse_post_trigger_s(d.pop("post_trigger_s", UNSET))

        dump_create = cls(
            note=note,
            pinned=pinned,
            post_trigger_s=post_trigger_s,
        )

        return dump_create
