from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

if TYPE_CHECKING:
    from ..models.dump_trigger import DumpTrigger
    from ..models.recorder_namespace import RecorderNamespace
    from ..models.recorder_status_suppressed import RecorderStatusSuppressed


T = TypeVar("T", bound="RecorderStatus")


@_attrs_define
class RecorderStatus:
    """`GET /v1/recorder`: what the recorder is, what it holds, and what it last
    saw.

        Attributes:
            dump_bytes (int): Bytes they occupy.
            dump_count (int): Dumps currently stored.
            dump_on_estop_at_start (bool): `[recorder] dump_on_estop_at_start`.
            dumps_dir (str): Where dumps are written.
            enabled (bool): Always `true` on a daemon that answers this route at all: a daemon
                with `[recorder] enabled = false` refuses every recorder route with
                `unavailable` rather than answering a status that says "off".
            last_trigger (DumpTrigger | None):
            log_tail_bytes (int): `[recorder] log_tail_bytes`.
            max_dumps (int): `[recorder] max_dumps`.
            max_total_bytes (int): `[recorder] max_total_bytes`.
            min_interval_s (int): `[recorder] min_interval_s`.
            namespaces (list[RecorderNamespace]): What the ring currently holds, per namespace, in the state log's own
                stable order. A namespace that has recorded nothing is absent.
            pinned_count (int): How many of them are pinned.
            post_trigger_s (int): `[recorder] post_trigger_s`.
            statelog_dir (None | str): The state-log root the ring is copied from, or `null` when no state
                log is running — in which case a dump carries the snapshot and the log
                tail and no ring.
            suppressed (RecorderStatusSuppressed): Triggers this daemon suppressed, by reason: `min_interval`,
                `all_pinned`, `failed`.
    """

    dump_bytes: int
    dump_count: int
    dump_on_estop_at_start: bool
    dumps_dir: str
    enabled: bool
    last_trigger: DumpTrigger | None
    log_tail_bytes: int
    max_dumps: int
    max_total_bytes: int
    min_interval_s: int
    namespaces: list[RecorderNamespace]
    pinned_count: int
    post_trigger_s: int
    statelog_dir: None | str
    suppressed: RecorderStatusSuppressed
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        from ..models.dump_trigger import DumpTrigger

        dump_bytes = self.dump_bytes

        dump_count = self.dump_count

        dump_on_estop_at_start = self.dump_on_estop_at_start

        dumps_dir = self.dumps_dir

        enabled = self.enabled

        last_trigger: dict[str, Any] | None
        if isinstance(self.last_trigger, DumpTrigger):
            last_trigger = self.last_trigger.to_dict()
        else:
            last_trigger = self.last_trigger

        log_tail_bytes = self.log_tail_bytes

        max_dumps = self.max_dumps

        max_total_bytes = self.max_total_bytes

        min_interval_s = self.min_interval_s

        namespaces = []
        for namespaces_item_data in self.namespaces:
            namespaces_item = namespaces_item_data.to_dict()
            namespaces.append(namespaces_item)

        pinned_count = self.pinned_count

        post_trigger_s = self.post_trigger_s

        statelog_dir: None | str
        statelog_dir = self.statelog_dir

        suppressed = self.suppressed.to_dict()

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "dump_bytes": dump_bytes,
                "dump_count": dump_count,
                "dump_on_estop_at_start": dump_on_estop_at_start,
                "dumps_dir": dumps_dir,
                "enabled": enabled,
                "last_trigger": last_trigger,
                "log_tail_bytes": log_tail_bytes,
                "max_dumps": max_dumps,
                "max_total_bytes": max_total_bytes,
                "min_interval_s": min_interval_s,
                "namespaces": namespaces,
                "pinned_count": pinned_count,
                "post_trigger_s": post_trigger_s,
                "statelog_dir": statelog_dir,
                "suppressed": suppressed,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.dump_trigger import DumpTrigger
        from ..models.recorder_namespace import RecorderNamespace
        from ..models.recorder_status_suppressed import (
            RecorderStatusSuppressed,
        )

        d = dict(src_dict)
        dump_bytes = d.pop("dump_bytes")

        dump_count = d.pop("dump_count")

        dump_on_estop_at_start = d.pop("dump_on_estop_at_start")

        dumps_dir = d.pop("dumps_dir")

        enabled = d.pop("enabled")

        def _parse_last_trigger(data: object) -> DumpTrigger | None:
            if data is None:
                return data
            try:
                if not isinstance(data, dict):
                    raise TypeError()
                last_trigger_type_1 = DumpTrigger.from_dict(data)

                return last_trigger_type_1
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            return cast(DumpTrigger | None, data)

        last_trigger = _parse_last_trigger(d.pop("last_trigger"))

        log_tail_bytes = d.pop("log_tail_bytes")

        max_dumps = d.pop("max_dumps")

        max_total_bytes = d.pop("max_total_bytes")

        min_interval_s = d.pop("min_interval_s")

        namespaces = []
        _namespaces = d.pop("namespaces")
        for namespaces_item_data in _namespaces:
            namespaces_item = RecorderNamespace.from_dict(namespaces_item_data)

            namespaces.append(namespaces_item)

        pinned_count = d.pop("pinned_count")

        post_trigger_s = d.pop("post_trigger_s")

        def _parse_statelog_dir(data: object) -> None | str:
            if data is None:
                return data
            return cast(None | str, data)

        statelog_dir = _parse_statelog_dir(d.pop("statelog_dir"))

        suppressed = RecorderStatusSuppressed.from_dict(d.pop("suppressed"))

        recorder_status = cls(
            dump_bytes=dump_bytes,
            dump_count=dump_count,
            dump_on_estop_at_start=dump_on_estop_at_start,
            dumps_dir=dumps_dir,
            enabled=enabled,
            last_trigger=last_trigger,
            log_tail_bytes=log_tail_bytes,
            max_dumps=max_dumps,
            max_total_bytes=max_total_bytes,
            min_interval_s=min_interval_s,
            namespaces=namespaces,
            pinned_count=pinned_count,
            post_trigger_s=post_trigger_s,
            statelog_dir=statelog_dir,
            suppressed=suppressed,
        )

        recorder_status.additional_properties = d
        return recorder_status

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
