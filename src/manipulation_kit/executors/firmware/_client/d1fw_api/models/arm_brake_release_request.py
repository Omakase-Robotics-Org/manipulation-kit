from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..types import UNSET, Unset

T = TypeVar("T", bound="ArmBrakeReleaseRequest")


@_attrs_define
class ArmBrakeReleaseRequest:
    """`POST /v1/arm/{side}/brake_release`.

    Attributes:
        confirm (str): Must be exactly `RELEASE_BRAKE`. Anything else is a `400`, and the body
            is required: the arm drops under gravity the moment the brakes open,
            and an empty POST is how a console button or a curl retry reaches a
            route by accident.
        seconds (float | None | Unset): How long the brakes stay open before the daemon engages them itself,
            in seconds, `1..=120`. Omitted is `30`. Sending the request again while
            released restarts the window.
    """

    confirm: str
    seconds: float | None | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        confirm = self.confirm

        seconds: float | None | Unset
        if isinstance(self.seconds, Unset):
            seconds = UNSET
        else:
            seconds = self.seconds

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "confirm": confirm,
            }
        )
        if seconds is not UNSET:
            field_dict["seconds"] = seconds

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        confirm = d.pop("confirm")

        def _parse_seconds(data: object) -> float | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(float | None | Unset, data)

        seconds = _parse_seconds(d.pop("seconds", UNSET))

        arm_brake_release_request = cls(
            confirm=confirm,
            seconds=seconds,
        )

        arm_brake_release_request.additional_properties = d
        return arm_brake_release_request

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
