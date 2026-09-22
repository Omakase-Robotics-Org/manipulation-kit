from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.vendor_mapping_save_state import VendorMappingSaveState

T = TypeVar("T", bound="VendorMappingObservation")


@_attrs_define
class VendorMappingObservation:
    """Immediate decision-state observation, independent of the service outcome.

    Attributes:
        auto_status (int | None): Fresh decision status, or null when unavailable.
        current_scene (None | str): Fresh scene, or null when unavailable.
        save_state (VendorMappingSaveState): Saving cannot be proved by service acknowledgment or decision mode alone.
        transition_observed (bool): Expected decision mode was observed; does not prove downstream mapping.
        work_mode (int | None): Fresh decision work mode, or null when unavailable.
    """

    auto_status: int | None
    current_scene: None | str
    save_state: VendorMappingSaveState
    transition_observed: bool
    work_mode: int | None
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        auto_status: int | None
        auto_status = self.auto_status

        current_scene: None | str
        current_scene = self.current_scene

        save_state = self.save_state.value

        transition_observed = self.transition_observed

        work_mode: int | None
        work_mode = self.work_mode

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "auto_status": auto_status,
                "current_scene": current_scene,
                "save_state": save_state,
                "transition_observed": transition_observed,
                "work_mode": work_mode,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)

        def _parse_auto_status(data: object) -> int | None:
            if data is None:
                return data
            return cast(int | None, data)

        auto_status = _parse_auto_status(d.pop("auto_status"))

        def _parse_current_scene(data: object) -> None | str:
            if data is None:
                return data
            return cast(None | str, data)

        current_scene = _parse_current_scene(d.pop("current_scene"))

        save_state = VendorMappingSaveState(d.pop("save_state"))

        transition_observed = d.pop("transition_observed")

        def _parse_work_mode(data: object) -> int | None:
            if data is None:
                return data
            return cast(int | None, data)

        work_mode = _parse_work_mode(d.pop("work_mode"))

        vendor_mapping_observation = cls(
            auto_status=auto_status,
            current_scene=current_scene,
            save_state=save_state,
            transition_observed=transition_observed,
            work_mode=work_mode,
        )

        vendor_mapping_observation.additional_properties = d
        return vendor_mapping_observation

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
