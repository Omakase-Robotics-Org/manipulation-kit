from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.mapping_jog_direction import MappingJogDirection
from ..models.mapping_jog_request_type_0_action import MappingJogRequestType0Action

if TYPE_CHECKING:
    from ..models.vendor_mapping_expected import VendorMappingExpected


T = TypeVar("T", bound="MappingJogRequestType0")


@_attrs_define
class MappingJogRequestType0:
    """Begin one hold after reviewing current scene and control token.

    Attributes:
        action (MappingJogRequestType0Action):
        direction (MappingJogDirection): One fixed motion axis per hold; no diagonal input.
        expected (VendorMappingExpected): Operator-observed mode precondition, checked again immediately before sending.
        expected_control_token (str): Opaque token from GET state; stop invalidates it even before start arrives.
        scene (str): Exact reported mapping scene.
        speed (float): Positive magnitude in metres/sec or radians/sec for the chosen axis.
    """

    action: MappingJogRequestType0Action
    direction: MappingJogDirection
    expected: VendorMappingExpected
    expected_control_token: str
    scene: str
    speed: float
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        action = self.action.value

        direction = self.direction.value

        expected = self.expected.to_dict()

        expected_control_token = self.expected_control_token

        scene = self.scene

        speed = self.speed

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "action": action,
                "direction": direction,
                "expected": expected,
                "expected_control_token": expected_control_token,
                "scene": scene,
                "speed": speed,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.vendor_mapping_expected import (
            VendorMappingExpected,
        )

        d = dict(src_dict)
        action = MappingJogRequestType0Action(d.pop("action"))

        direction = MappingJogDirection(d.pop("direction"))

        expected = VendorMappingExpected.from_dict(d.pop("expected"))

        expected_control_token = d.pop("expected_control_token")

        scene = d.pop("scene")

        speed = d.pop("speed")

        mapping_jog_request_type_0 = cls(
            action=action,
            direction=direction,
            expected=expected,
            expected_control_token=expected_control_token,
            scene=scene,
            speed=speed,
        )

        mapping_jog_request_type_0.additional_properties = d
        return mapping_jog_request_type_0

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
