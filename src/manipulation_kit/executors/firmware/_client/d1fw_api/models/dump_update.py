from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from typing_extensions import Self

from ..types import UNSET, Unset

T = TypeVar("T", bound="DumpUpdate")


@_attrs_define
class DumpUpdate:
    """`PATCH /v1/recorder/dumps/{id}`: the fields of one dump an operator may
    change.

    Which dump is not in the body: it is the `{id}` of the path, because a
    dump is a resource this daemon owns and a PATCH addresses the resource it
    is sent to. A body that could name a second dump would be a body that can
    disagree with the path.

        Attributes:
            note (None | str | Unset): The note to store. Omitted or `null` leaves the existing note alone;
                an EMPTY STRING clears it, which is how a note is removed without a
                second field to say so.
            pinned (bool | None | Unset): Whether retention may evict this dump. Omitted leaves it alone.
    """

    note: None | str | Unset = UNSET
    pinned: bool | None | Unset = UNSET

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

        field_dict: dict[str, Any] = {}

        field_dict.update({})
        if note is not UNSET:
            field_dict["note"] = note
        if pinned is not UNSET:
            field_dict["pinned"] = pinned

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

        dump_update = cls(
            note=note,
            pinned=pinned,
        )

        return dump_update
