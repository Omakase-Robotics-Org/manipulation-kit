from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

T = TypeVar("T", bound="HandState")


@_attrs_define
class HandState:
    """One sampled snapshot of a hand.

    Attributes:
        all_enabled (bool): Whether the daemon believes every axis is enabled.
        enabled (list[bool]): Per-axis enabled flags, as the hand reports them.
        error_code (int): The most recent device error record: `0` means none. On the O30 this
            is the hand-wide MI `0x4F` communication error; on the DH116S it is
            the first faulted axis's code.
        faults (list[str]): Human descriptions of every fault currently reported.
        features (list[str]): Capability tokens, repeated here so a consumer that only polls state
            does not have to fetch capabilities separately.
        live (bool): Whether `positions` came from a reading taken for THIS call, rather
            than from the last one the command path happened to take.

            A read on these hands is not free — it competes with the command path
            for the arm's single passthrough channel — so a busy hand answers
            with its last sample and says so here, exactly as the gripper's
            report does.
        model (str): Which model answered.
        positions (list[float]): Per-axis measured position as a fraction of the axis's range.
        positions_wire (list[int]): Per-axis measured position in the raw vendor unit.
        side (str): Which arm side it is mounted on.
    """

    all_enabled: bool
    enabled: list[bool]
    error_code: int
    faults: list[str]
    features: list[str]
    live: bool
    model: str
    positions: list[float]
    positions_wire: list[int]
    side: str
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        all_enabled = self.all_enabled

        enabled = self.enabled

        error_code = self.error_code

        faults = self.faults

        features = self.features

        live = self.live

        model = self.model

        positions = self.positions

        positions_wire = self.positions_wire

        side = self.side

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "all_enabled": all_enabled,
                "enabled": enabled,
                "error_code": error_code,
                "faults": faults,
                "features": features,
                "live": live,
                "model": model,
                "positions": positions,
                "positions_wire": positions_wire,
                "side": side,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        all_enabled = d.pop("all_enabled")

        enabled = cast(list[bool], d.pop("enabled"))

        error_code = d.pop("error_code")

        faults = cast(list[str], d.pop("faults"))

        features = cast(list[str], d.pop("features"))

        live = d.pop("live")

        model = d.pop("model")

        positions = cast(list[float], d.pop("positions"))

        positions_wire = cast(list[int], d.pop("positions_wire"))

        side = d.pop("side")

        hand_state = cls(
            all_enabled=all_enabled,
            enabled=enabled,
            error_code=error_code,
            faults=faults,
            features=features,
            live=live,
            model=model,
            positions=positions,
            positions_wire=positions_wire,
            side=side,
        )

        hand_state.additional_properties = d
        return hand_state

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
