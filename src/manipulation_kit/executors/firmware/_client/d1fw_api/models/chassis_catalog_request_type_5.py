from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.chassis_catalog_request_type_5_kind import ChassisCatalogRequestType5Kind

T = TypeVar("T", bound="ChassisCatalogRequestType5")


@_attrs_define
class ChassisCatalogRequestType5:
    """Whether the vendor web application currently holds its own connection
    to the base's ROS side: `{"connected": <bool>}`.

    Computed inside the web application from three in-process checks, so
    it touches neither the chassis nor the robot. It is the cheapest
    statement the web application can make about its link to the base, and
    it is a DIFFERENT fact from the daemon's own reachability: a web
    application that answers this read with `false` is up while its ROS
    side is not.

    Served only by the `corona-2026` mobile-base firmware generation's web
    application; a `gs-2026` base refuses this read.

        Attributes:
            kind (ChassisCatalogRequestType5Kind):
    """

    kind: ChassisCatalogRequestType5Kind
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        kind = self.kind.value

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "kind": kind,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        kind = ChassisCatalogRequestType5Kind(d.pop("kind"))

        chassis_catalog_request_type_5 = cls(
            kind=kind,
        )

        chassis_catalog_request_type_5.additional_properties = d
        return chassis_catalog_request_type_5

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
