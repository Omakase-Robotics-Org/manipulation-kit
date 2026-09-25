from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

if TYPE_CHECKING:
    from ..models.dump_namespace import DumpNamespace


T = TypeVar("T", bound="DumpPreTrigger")


@_attrs_define
class DumpPreTrigger:
    """What the ring gave this dump: the history that existed BEFORE the trigger.

    This is the part of a dump that cannot be reconstructed afterwards, so the
    manifest states its extent rather than leaving a reader to measure it.

        Attributes:
            namespaces (list[DumpNamespace]): One entry per namespace that had anything recorded, in the state log's
                own stable namespace order.
            statelog_dir (None | str): The state-log root the ring was copied from, or `null` when no
                recorder was running and the dump carries no ring at all.
    """

    namespaces: list[DumpNamespace]
    statelog_dir: None | str
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        namespaces = []
        for namespaces_item_data in self.namespaces:
            namespaces_item = namespaces_item_data.to_dict()
            namespaces.append(namespaces_item)

        statelog_dir: None | str
        statelog_dir = self.statelog_dir

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "namespaces": namespaces,
                "statelog_dir": statelog_dir,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.dump_namespace import DumpNamespace

        d = dict(src_dict)
        namespaces = []
        _namespaces = d.pop("namespaces")
        for namespaces_item_data in _namespaces:
            namespaces_item = DumpNamespace.from_dict(namespaces_item_data)

            namespaces.append(namespaces_item)

        def _parse_statelog_dir(data: object) -> None | str:
            if data is None:
                return data
            return cast(None | str, data)

        statelog_dir = _parse_statelog_dir(d.pop("statelog_dir"))

        dump_pre_trigger = cls(
            namespaces=namespaces,
            statelog_dir=statelog_dir,
        )

        dump_pre_trigger.additional_properties = d
        return dump_pre_trigger

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
