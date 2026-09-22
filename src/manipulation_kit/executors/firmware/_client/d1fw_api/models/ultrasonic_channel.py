from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.ultrasonic_radiation import UltrasonicRadiation
from ..types import UNSET, Unset

T = TypeVar("T", bound="UltrasonicChannel")


@_attrs_define
class UltrasonicChannel:
    """One channel of the mobile base's ultrasonic proximity ring.

    The base reports a channel that received no echo, or whose target is
    beyond its range, as the sentinel distance `100.0` metres rather than as a
    failure. That sentinel is NOT a distance, so [`UltrasonicChannel::range_m`]
    is `null` for such a channel and the untouched number stays on
    [`UltrasonicChannel::raw_range_m`] where an operator checking the hardware
    can still see exactly what the base said.

        Attributes:
            age_ms (int): How long ago this channel's reading was taken, in milliseconds.

                `0` means the channel was in the newest frame the daemon read. A
                larger value means it was not, and this is still the last thing that
                channel said: the base's own example capture shows three channels on a
                chassis configured for four, so a channel dropping out of the array is
                a thing the service does rather than an error.
            index (int): The vendor's channel `id`, counting from 0.
            radiation (UltrasonicRadiation): What kind of emitter one ultrasonic-ring channel is, from the vendor's
                `radiation_type` field.
            raw_range_m (float): The number the base sent, untouched, including the `100.0` sentinel.
            valid (bool): Whether this channel's reading is a real distance, by the vendor's own
                rule `range < max_range`.
            field_of_view_rad (float | None | Unset): The channel's beam width in radians, or `null` when the frame carried
                no such field.
            max_range_m (float | None | Unset): The channel's maximum range in metres, or `null` when the frame
                carried no such field. This is the bound `valid` is tested against, so
                a channel with no bound is never valid.
            min_range_m (float | None | Unset): The channel's minimum range in metres, or `null` when the frame
                carried no such field.

                A reading below this bound is still reported as valid: on a proximity
                sensor that is an object closer than the transducer can measure, which
                is the ring's most important answer, not a fault.
            range_m (float | None | Unset): The measured distance in metres, or `null` when this channel reported
                no usable echo. Never the `100.0` sentinel and never a substituted 0.
    """

    age_ms: int
    index: int
    radiation: UltrasonicRadiation
    raw_range_m: float
    valid: bool
    field_of_view_rad: float | None | Unset = UNSET
    max_range_m: float | None | Unset = UNSET
    min_range_m: float | None | Unset = UNSET
    range_m: float | None | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        age_ms = self.age_ms

        index = self.index

        radiation = self.radiation.value

        raw_range_m = self.raw_range_m

        valid = self.valid

        field_of_view_rad: float | None | Unset
        if isinstance(self.field_of_view_rad, Unset):
            field_of_view_rad = UNSET
        else:
            field_of_view_rad = self.field_of_view_rad

        max_range_m: float | None | Unset
        if isinstance(self.max_range_m, Unset):
            max_range_m = UNSET
        else:
            max_range_m = self.max_range_m

        min_range_m: float | None | Unset
        if isinstance(self.min_range_m, Unset):
            min_range_m = UNSET
        else:
            min_range_m = self.min_range_m

        range_m: float | None | Unset
        if isinstance(self.range_m, Unset):
            range_m = UNSET
        else:
            range_m = self.range_m

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "age_ms": age_ms,
                "index": index,
                "radiation": radiation,
                "raw_range_m": raw_range_m,
                "valid": valid,
            }
        )
        if field_of_view_rad is not UNSET:
            field_dict["field_of_view_rad"] = field_of_view_rad
        if max_range_m is not UNSET:
            field_dict["max_range_m"] = max_range_m
        if min_range_m is not UNSET:
            field_dict["min_range_m"] = min_range_m
        if range_m is not UNSET:
            field_dict["range_m"] = range_m

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        age_ms = d.pop("age_ms")

        index = d.pop("index")

        radiation = UltrasonicRadiation(d.pop("radiation"))

        raw_range_m = d.pop("raw_range_m")

        valid = d.pop("valid")

        def _parse_field_of_view_rad(data: object) -> float | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(float | None | Unset, data)

        field_of_view_rad = _parse_field_of_view_rad(d.pop("field_of_view_rad", UNSET))

        def _parse_max_range_m(data: object) -> float | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(float | None | Unset, data)

        max_range_m = _parse_max_range_m(d.pop("max_range_m", UNSET))

        def _parse_min_range_m(data: object) -> float | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(float | None | Unset, data)

        min_range_m = _parse_min_range_m(d.pop("min_range_m", UNSET))

        def _parse_range_m(data: object) -> float | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(float | None | Unset, data)

        range_m = _parse_range_m(d.pop("range_m", UNSET))

        ultrasonic_channel = cls(
            age_ms=age_ms,
            index=index,
            radiation=radiation,
            raw_range_m=raw_range_m,
            valid=valid,
            field_of_view_rad=field_of_view_rad,
            max_range_m=max_range_m,
            min_range_m=min_range_m,
            range_m=range_m,
        )

        ultrasonic_channel.additional_properties = d
        return ultrasonic_channel

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
