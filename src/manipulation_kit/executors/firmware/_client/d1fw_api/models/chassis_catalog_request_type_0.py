from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.chassis_catalog_request_type_0_kind import ChassisCatalogRequestType0Kind

T = TypeVar("T", bound="ChassisCatalogRequestType0")


@_attrs_define
class ChassisCatalogRequestType0:
    """Saved tasks for an explicitly named scene.

    Attributes:
        kind (ChassisCatalogRequestType0Kind):
        scene (str): Exact saved scene name.
    """

    kind: ChassisCatalogRequestType0Kind
    scene: str
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        kind = self.kind.value

        scene = self.scene

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "kind": kind,
                "scene": scene,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        kind = ChassisCatalogRequestType0Kind(d.pop("kind"))

        scene = d.pop("scene")

        chassis_catalog_request_type_0 = cls(
            kind=kind,
            scene=scene,
        )

        chassis_catalog_request_type_0.additional_properties = d
        return chassis_catalog_request_type_0

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
