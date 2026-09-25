from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

T = TypeVar("T", bound="ChargeDock")


@_attrs_define
class ChargeDock:
    """The charging-dock scene and waypoint written through the map resource's
    `charging_dock` sub-resource, later used by `charge` (vendor
    `sendChargeCmd`).

        Attributes:
            point_id (str): Charging waypoint identifier, e.g. `"0000"`.
            scene_name (str): Charging map scene name, e.g. `"208_2_omakasemap1"`.
    """

    point_id: str
    scene_name: str
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        point_id = self.point_id

        scene_name = self.scene_name

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "point_id": point_id,
                "scene_name": scene_name,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        point_id = d.pop("point_id")

        scene_name = d.pop("scene_name")

        charge_dock = cls(
            point_id=point_id,
            scene_name=scene_name,
        )

        charge_dock.additional_properties = d
        return charge_dock

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
