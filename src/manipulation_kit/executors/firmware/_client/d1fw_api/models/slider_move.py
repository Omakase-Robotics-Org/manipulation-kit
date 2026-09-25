from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..types import UNSET, Unset

T = TypeVar("T", bound="SliderMove")


@_attrs_define
class SliderMove:
    """A requested slider move: one absolute height, or one small relative
    amount.

    Exactly one of `height_m` and `delta_m` is given. Neither and both are
    both rejected with `400` rather than resolved by a precedence rule: a
    caller that sent two targets did not mean one of them, and a caller that
    sent none named no move at all.

    `delta_m` is the commissioning form. It is admitted only on a daemon
    started with `[slider] commissioning = true`, it is capped at
    [`crate::safety::SLIDER_COMMISSIONING_STEP_MAX_M`] per call, and it runs
    at a fixed low speed — so `speed_ratio` may not be sent with it. It is on
    this verb rather than on a new one because on the LD2-RS's wire the two
    are the same PR path write with a different mode word (`0x0041` instead
    of `0x0001`), so a separate verb would have been a second name for one
    operation.

        Attributes:
            wait (bool): Wait for the backend to report completion when true.
            delta_m (float | None | Unset): A small relative amount in metres, measured from where the carriage
                stands now; negative is down. Commissioning only.
            height_m (float | None | Unset): Absolute target height in metres, measured from the drive's origin.

                Optional in the schema only because `delta_m` is the alternative; a
                body carrying neither is refused.
            speed_ratio (float | None | Unset): Optional move speed as a normalized ratio in `0.0..=1.0`: a fraction of
                the slider's full speed, the same convention the arm's `vel_ratio`
                uses (a fraction of full speed, never a percentage). `None` (the field
                absent from the request body) keeps the backend's default speed, so an
                existing caller is unchanged. Out-of-range values are rejected with
                `400`. It may not be combined with `delta_m`, which always moves at a
                fixed low commissioning speed.
    """

    wait: bool
    delta_m: float | None | Unset = UNSET
    height_m: float | None | Unset = UNSET
    speed_ratio: float | None | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        wait = self.wait

        delta_m: float | None | Unset
        if isinstance(self.delta_m, Unset):
            delta_m = UNSET
        else:
            delta_m = self.delta_m

        height_m: float | None | Unset
        if isinstance(self.height_m, Unset):
            height_m = UNSET
        else:
            height_m = self.height_m

        speed_ratio: float | None | Unset
        if isinstance(self.speed_ratio, Unset):
            speed_ratio = UNSET
        else:
            speed_ratio = self.speed_ratio

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "wait": wait,
            }
        )
        if delta_m is not UNSET:
            field_dict["delta_m"] = delta_m
        if height_m is not UNSET:
            field_dict["height_m"] = height_m
        if speed_ratio is not UNSET:
            field_dict["speed_ratio"] = speed_ratio

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        wait = d.pop("wait")

        def _parse_delta_m(data: object) -> float | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(float | None | Unset, data)

        delta_m = _parse_delta_m(d.pop("delta_m", UNSET))

        def _parse_height_m(data: object) -> float | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(float | None | Unset, data)

        height_m = _parse_height_m(d.pop("height_m", UNSET))

        def _parse_speed_ratio(data: object) -> float | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(float | None | Unset, data)

        speed_ratio = _parse_speed_ratio(d.pop("speed_ratio", UNSET))

        slider_move = cls(
            wait=wait,
            delta_m=delta_m,
            height_m=height_m,
            speed_ratio=speed_ratio,
        )

        slider_move.additional_properties = d
        return slider_move

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
