from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

if TYPE_CHECKING:
    from ..models.dump_file import DumpFile
    from ..models.dump_pre_trigger import DumpPreTrigger
    from ..models.dump_trigger import DumpTrigger


T = TypeVar("T", bound="DumpManifest")


@_attrs_define
class DumpManifest:
    """A dump's `manifest.json`, which is also what the API answers with.

    One type for the file and the wire on purpose: the file IS the record, and
    a second shape for the API would be a second thing to keep true.

        Attributes:
            bytes_ (int): The sum of `files[].bytes`; what this dump costs the retention budget.
            complete (bool): Whether the second phase finished. `false` means the daemon was
                stopped between the trigger and the copy: `snapshot.json` is there and
                the ring is not.
            created_ts (str): When the dump was created, UTC RFC 3339 with millisecond precision.
            daemon_version (str): The daemon version that wrote it.
            files (list[DumpFile]): Every file in the dump, `manifest.json` itself excluded.
            id (str): The dump's identifier and its directory name: a sortable UTC timestamp
                and a short suffix, e.g. `20260920T060312Z-a1b2`.
            note (None | str): An operator's note, or `null`.
            pinned (bool): Whether retention may evict this dump. A pinned dump never is.
            post_trigger_s (int): Seconds waited after the trigger before the ring was copied.
            pre_trigger (DumpPreTrigger): What the ring gave this dump: the history that existed BEFORE the trigger.

                This is the part of a dump that cannot be reconstructed afterwards, so the
                manifest states its extent rather than leaving a reader to measure it.
            session (str): The daemon run that wrote it — the same token the state log's `meta`
                records carry, so a dump and a ring can be tied to one run.
            trigger (DumpTrigger): One trigger: what raised it, when, and what it saw.
            v (int): Manifest version; [`MANIFEST_VERSION`] for anything this daemon wrote.
    """

    bytes_: int
    complete: bool
    created_ts: str
    daemon_version: str
    files: list[DumpFile]
    id: str
    note: None | str
    pinned: bool
    post_trigger_s: int
    pre_trigger: DumpPreTrigger
    session: str
    trigger: DumpTrigger
    v: int
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        bytes_ = self.bytes_

        complete = self.complete

        created_ts = self.created_ts

        daemon_version = self.daemon_version

        files = []
        for files_item_data in self.files:
            files_item = files_item_data.to_dict()
            files.append(files_item)

        id = self.id

        note: None | str
        note = self.note

        pinned = self.pinned

        post_trigger_s = self.post_trigger_s

        pre_trigger = self.pre_trigger.to_dict()

        session = self.session

        trigger = self.trigger.to_dict()

        v = self.v

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "bytes": bytes_,
                "complete": complete,
                "created_ts": created_ts,
                "daemon_version": daemon_version,
                "files": files,
                "id": id,
                "note": note,
                "pinned": pinned,
                "post_trigger_s": post_trigger_s,
                "pre_trigger": pre_trigger,
                "session": session,
                "trigger": trigger,
                "v": v,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.dump_file import DumpFile
        from ..models.dump_pre_trigger import DumpPreTrigger
        from ..models.dump_trigger import DumpTrigger

        d = dict(src_dict)
        bytes_ = d.pop("bytes")

        complete = d.pop("complete")

        created_ts = d.pop("created_ts")

        daemon_version = d.pop("daemon_version")

        files = []
        _files = d.pop("files")
        for files_item_data in _files:
            files_item = DumpFile.from_dict(files_item_data)

            files.append(files_item)

        id = d.pop("id")

        def _parse_note(data: object) -> None | str:
            if data is None:
                return data
            return cast(None | str, data)

        note = _parse_note(d.pop("note"))

        pinned = d.pop("pinned")

        post_trigger_s = d.pop("post_trigger_s")

        pre_trigger = DumpPreTrigger.from_dict(d.pop("pre_trigger"))

        session = d.pop("session")

        trigger = DumpTrigger.from_dict(d.pop("trigger"))

        v = d.pop("v")

        dump_manifest = cls(
            bytes_=bytes_,
            complete=complete,
            created_ts=created_ts,
            daemon_version=daemon_version,
            files=files,
            id=id,
            note=note,
            pinned=pinned,
            post_trigger_s=post_trigger_s,
            pre_trigger=pre_trigger,
            session=session,
            trigger=trigger,
            v=v,
        )

        dump_manifest.additional_properties = d
        return dump_manifest

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
