from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

if TYPE_CHECKING:
    from ..models.chassis_catalog_request_type_0 import ChassisCatalogRequestType0
    from ..models.chassis_catalog_request_type_1 import ChassisCatalogRequestType1
    from ..models.chassis_catalog_request_type_2 import ChassisCatalogRequestType2
    from ..models.chassis_catalog_request_type_3 import ChassisCatalogRequestType3
    from ..models.chassis_catalog_request_type_4 import ChassisCatalogRequestType4
    from ..models.chassis_catalog_request_type_5 import ChassisCatalogRequestType5
    from ..models.chassis_catalog_request_type_6 import ChassisCatalogRequestType6


T = TypeVar("T", bound="ChassisCatalogSnapshot")


@_attrs_define
class ChassisCatalogSnapshot:
    """A received vendor document. Its schema may differ by firmware generation.

    Attributes:
        received_at_unix_ms (int): Unix milliseconds when the daemon received the document.
        request (ChassisCatalogRequestType0 | ChassisCatalogRequestType1 | ChassisCatalogRequestType2 |
            ChassisCatalogRequestType3 | ChassisCatalogRequestType4 | ChassisCatalogRequestType5 |
            ChassisCatalogRequestType6): Exactly seven supported reads. No caller-selected endpoint is accepted.

            This is where a fixed, parameterless or identifier-selected vendor
            DOCUMENT read belongs, rather than a route of its own: the daemon's own
            chassis routes publish the daemon's model of the base (its state, its
            navigation, its settings), while these publish a vendor document verbatim,
            with its provenance, for a consumer that needs the vendor's own wording.
            `RosStatus` and `ChargingStatToday` are read-only reads of exactly that
            shape, which is why they are variants here.
        source (str): Fixed vendor HTTP endpoint that supplied the value.
        value (Any): Complete decoded result, including unknown fields and explicit null.
    """

    received_at_unix_ms: int
    request: (
        ChassisCatalogRequestType0
        | ChassisCatalogRequestType1
        | ChassisCatalogRequestType2
        | ChassisCatalogRequestType3
        | ChassisCatalogRequestType4
        | ChassisCatalogRequestType5
        | ChassisCatalogRequestType6
    )
    source: str
    value: Any
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        from ..models.chassis_catalog_request_type_0 import (
            ChassisCatalogRequestType0,
        )
        from ..models.chassis_catalog_request_type_1 import (
            ChassisCatalogRequestType1,
        )
        from ..models.chassis_catalog_request_type_2 import (
            ChassisCatalogRequestType2,
        )
        from ..models.chassis_catalog_request_type_3 import (
            ChassisCatalogRequestType3,
        )
        from ..models.chassis_catalog_request_type_4 import (
            ChassisCatalogRequestType4,
        )
        from ..models.chassis_catalog_request_type_5 import (
            ChassisCatalogRequestType5,
        )

        received_at_unix_ms = self.received_at_unix_ms

        request: dict[str, Any]
        if (
            isinstance(self.request, ChassisCatalogRequestType0)
            or isinstance(self.request, ChassisCatalogRequestType1)
            or isinstance(self.request, ChassisCatalogRequestType2)
            or isinstance(self.request, ChassisCatalogRequestType3)
            or isinstance(self.request, ChassisCatalogRequestType4)
            or isinstance(self.request, ChassisCatalogRequestType5)
        ):
            request = self.request.to_dict()
        else:
            request = self.request.to_dict()

        source = self.source

        value = self.value

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "received_at_unix_ms": received_at_unix_ms,
                "request": request,
                "source": source,
                "value": value,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.chassis_catalog_request_type_0 import (
            ChassisCatalogRequestType0,
        )
        from ..models.chassis_catalog_request_type_1 import (
            ChassisCatalogRequestType1,
        )
        from ..models.chassis_catalog_request_type_2 import (
            ChassisCatalogRequestType2,
        )
        from ..models.chassis_catalog_request_type_3 import (
            ChassisCatalogRequestType3,
        )
        from ..models.chassis_catalog_request_type_4 import (
            ChassisCatalogRequestType4,
        )
        from ..models.chassis_catalog_request_type_5 import (
            ChassisCatalogRequestType5,
        )
        from ..models.chassis_catalog_request_type_6 import (
            ChassisCatalogRequestType6,
        )

        d = dict(src_dict)
        received_at_unix_ms = d.pop("received_at_unix_ms")

        def _parse_request(
            data: object,
        ) -> (
            ChassisCatalogRequestType0
            | ChassisCatalogRequestType1
            | ChassisCatalogRequestType2
            | ChassisCatalogRequestType3
            | ChassisCatalogRequestType4
            | ChassisCatalogRequestType5
            | ChassisCatalogRequestType6
        ):
            try:
                if not isinstance(data, dict):
                    raise TypeError()
                componentsschemas_chassis_catalog_request_type_0 = (
                    ChassisCatalogRequestType0.from_dict(data)
                )

                return componentsschemas_chassis_catalog_request_type_0
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            try:
                if not isinstance(data, dict):
                    raise TypeError()
                componentsschemas_chassis_catalog_request_type_1 = (
                    ChassisCatalogRequestType1.from_dict(data)
                )

                return componentsschemas_chassis_catalog_request_type_1
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            try:
                if not isinstance(data, dict):
                    raise TypeError()
                componentsschemas_chassis_catalog_request_type_2 = (
                    ChassisCatalogRequestType2.from_dict(data)
                )

                return componentsschemas_chassis_catalog_request_type_2
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            try:
                if not isinstance(data, dict):
                    raise TypeError()
                componentsschemas_chassis_catalog_request_type_3 = (
                    ChassisCatalogRequestType3.from_dict(data)
                )

                return componentsschemas_chassis_catalog_request_type_3
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            try:
                if not isinstance(data, dict):
                    raise TypeError()
                componentsschemas_chassis_catalog_request_type_4 = (
                    ChassisCatalogRequestType4.from_dict(data)
                )

                return componentsschemas_chassis_catalog_request_type_4
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            try:
                if not isinstance(data, dict):
                    raise TypeError()
                componentsschemas_chassis_catalog_request_type_5 = (
                    ChassisCatalogRequestType5.from_dict(data)
                )

                return componentsschemas_chassis_catalog_request_type_5
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            if not isinstance(data, dict):
                raise TypeError()
            componentsschemas_chassis_catalog_request_type_6 = (
                ChassisCatalogRequestType6.from_dict(data)
            )

            return componentsschemas_chassis_catalog_request_type_6

        request = _parse_request(d.pop("request"))

        source = d.pop("source")

        value = d.pop("value")

        chassis_catalog_snapshot = cls(
            received_at_unix_ms=received_at_unix_ms,
            request=request,
            source=source,
            value=value,
        )

        chassis_catalog_snapshot.additional_properties = d
        return chassis_catalog_snapshot

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
