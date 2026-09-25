from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

T = TypeVar("T", bound="ChassisPlansSnapshot")


@_attrs_define
class ChassisPlansSnapshot:
    """Raw storage verified through the bounded scene export.

    Attributes:
        plans (Any): Raw JSON collection; unknown fields are retained for inspection.
        points (Any): Fresh raw point list for this scene.
        received_at_unix_ms (int): UTC Unix milliseconds when all reads completed.
        revision (str): Opaque token covers scene and entire raw plan collection.
        scene (str): Explicit selected scene.
        source (str): Fixed endpoint and bounded archive member provenance.
        write_block (None | str): Reason writes cannot preserve this collection, or null if representable.
    """

    plans: Any
    points: Any
    received_at_unix_ms: int
    revision: str
    scene: str
    source: str
    write_block: None | str
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        plans = self.plans

        points = self.points

        received_at_unix_ms = self.received_at_unix_ms

        revision = self.revision

        scene = self.scene

        source = self.source

        write_block: None | str
        write_block = self.write_block

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "plans": plans,
                "points": points,
                "received_at_unix_ms": received_at_unix_ms,
                "revision": revision,
                "scene": scene,
                "source": source,
                "write_block": write_block,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        plans = d.pop("plans")

        points = d.pop("points")

        received_at_unix_ms = d.pop("received_at_unix_ms")

        revision = d.pop("revision")

        scene = d.pop("scene")

        source = d.pop("source")

        def _parse_write_block(data: object) -> None | str:
            if data is None:
                return data
            return cast(None | str, data)

        write_block = _parse_write_block(d.pop("write_block"))

        chassis_plans_snapshot = cls(
            plans=plans,
            points=points,
            received_at_unix_ms=received_at_unix_ms,
            revision=revision,
            scene=scene,
            source=source,
            write_block=write_block,
        )

        chassis_plans_snapshot.additional_properties = d
        return chassis_plans_snapshot

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
