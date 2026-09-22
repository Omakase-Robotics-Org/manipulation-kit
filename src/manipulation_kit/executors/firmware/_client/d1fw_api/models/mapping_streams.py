from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.mapping_map_source import MappingMapSource
from ..models.vendor_ros_connection import VendorRosConnection

if TYPE_CHECKING:
    from ..models.mapping_grid_reading import MappingGridReading
    from ..models.mapping_scan_reading import MappingScanReading


T = TypeVar("T", bound="MappingStreams")


@_attrs_define
class MappingStreams:
    """Separate read-only bridge stream; never issues command or publish messages.

    Attributes:
        connection (VendorRosConnection): Latest transport state. Connected does not imply that topics have arrived.
        last_error (None | str): Current connection diagnostic.
        map_ (MappingGridReading): One independently aged stream; payload is null when client's revision matches.
        map_modify (MappingGridReading): One independently aged stream; payload is null when client's revision matches.
        map_source (MappingMapSource): Which of the two occupancy-grid readings describes the base's map now.
        max_map_cells (int): Application cell budget, not a manufacturer maximum.
        max_scan_samples (int): Application ray budget, not a manufacturer maximum.
        scan (MappingScanReading): One independently aged stream; payload is null when client's revision matches.
        source (str): Always vendor_ros_mapping_streams.
        stale_after_ms (int): Receipt freshness threshold; application policy, not vendor guarantee.
        stream_id (str): Reader identity, stable across reconnects but different after daemon restart.
    """

    connection: VendorRosConnection
    last_error: None | str
    map_: MappingGridReading
    map_modify: MappingGridReading
    map_source: MappingMapSource
    max_map_cells: int
    max_scan_samples: int
    scan: MappingScanReading
    source: str
    stale_after_ms: int
    stream_id: str
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        connection = self.connection.value

        last_error: None | str
        last_error = self.last_error

        map_ = self.map_.to_dict()

        map_modify = self.map_modify.to_dict()

        map_source = self.map_source.value

        max_map_cells = self.max_map_cells

        max_scan_samples = self.max_scan_samples

        scan = self.scan.to_dict()

        source = self.source

        stale_after_ms = self.stale_after_ms

        stream_id = self.stream_id

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "connection": connection,
                "last_error": last_error,
                "map": map_,
                "map_modify": map_modify,
                "map_source": map_source,
                "max_map_cells": max_map_cells,
                "max_scan_samples": max_scan_samples,
                "scan": scan,
                "source": source,
                "stale_after_ms": stale_after_ms,
                "stream_id": stream_id,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.mapping_grid_reading import MappingGridReading
        from ..models.mapping_scan_reading import MappingScanReading

        d = dict(src_dict)
        connection = VendorRosConnection(d.pop("connection"))

        def _parse_last_error(data: object) -> None | str:
            if data is None:
                return data
            return cast(None | str, data)

        last_error = _parse_last_error(d.pop("last_error"))

        map_ = MappingGridReading.from_dict(d.pop("map"))

        map_modify = MappingGridReading.from_dict(d.pop("map_modify"))

        map_source = MappingMapSource(d.pop("map_source"))

        max_map_cells = d.pop("max_map_cells")

        max_scan_samples = d.pop("max_scan_samples")

        scan = MappingScanReading.from_dict(d.pop("scan"))

        source = d.pop("source")

        stale_after_ms = d.pop("stale_after_ms")

        stream_id = d.pop("stream_id")

        mapping_streams = cls(
            connection=connection,
            last_error=last_error,
            map_=map_,
            map_modify=map_modify,
            map_source=map_source,
            max_map_cells=max_map_cells,
            max_scan_samples=max_scan_samples,
            scan=scan,
            source=source,
            stale_after_ms=stale_after_ms,
            stream_id=stream_id,
        )

        mapping_streams.additional_properties = d
        return mapping_streams

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
