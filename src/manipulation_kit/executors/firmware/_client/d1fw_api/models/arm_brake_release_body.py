from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..types import UNSET, Unset

T = TypeVar("T", bound="ArmBrakeReleaseBody")


@_attrs_define
class ArmBrakeReleaseBody:
    """
    Attributes:
        confirm (str): Must be exactly `RELEASE_BRAKE`. Anything else is a `400`, and the body
            is required: the arm drops under gravity the moment the brakes open,
            and an empty POST is how a console button or a curl retry reaches a
            route by accident.
        seconds (float | None | Unset): How long the brakes stay open before the daemon engages them itself,
            in seconds, `1..=120`. Omitted is `30`. Sending the request again while
            released restarts the window.
        holder (str | Unset): The arm lease holder issuing this command. Required only while somebody holds the lease:
            with no lease held the field is ignored, and with one held a request whose `holder` does not match is refused
            with 409.
    """

    confirm: str
    seconds: float | None | Unset = UNSET
    holder: str | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        confirm = self.confirm

        seconds: float | None | Unset
        if isinstance(self.seconds, Unset):
            seconds = UNSET
        else:
            seconds = self.seconds

        holder = self.holder

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "confirm": confirm,
            }
        )
        if seconds is not UNSET:
            field_dict["seconds"] = seconds
        if holder is not UNSET:
            field_dict["holder"] = holder

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

        holder = d.pop("holder", UNSET)

        arm_brake_release_body = cls(
            confirm=confirm,
            seconds=seconds,
            holder=holder,
        )

        arm_brake_release_body.additional_properties = d
        return arm_brake_release_body

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
