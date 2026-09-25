from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.zero_reference import ZeroReference
from ..models.zero_verification import ZeroVerification
from ..types import UNSET, Unset

if TYPE_CHECKING:
    from ..models.advisory import Advisory
    from ..models.slider_travel_limits import SliderTravelLimits


T = TypeVar("T", bound="SliderState")


@_attrs_define
class SliderState:
    """Slider feedback.

    Attributes:
        alarm (bool): Whether the slider reports an alarm.
        alarm_code (int): The masked 12-bit raw alarm register value (see
            `d1fw_proto_slider::AlarmCode`). `0` when the drive reports no alarm,
            and also `0` when `comms_ok` is false since a failed read carries no
            trustworthy register content.
        alarm_text (str): Human-readable alarm text.
        comms_ok (bool): Whether communications with the slider are healthy.
        height_m (float): Current height in metres.
        moving (bool): Whether the slider is moving.
        travel_max_m (float): The upper bound of the slider's usable travel in metres — the highest
            `height_m` a move may target. Sourced from the daemon's `[slider]
            travel_max_m` configuration (default `0.30` on a D1) so a console can
            present the real limit instead of hardcoding one. The lower bound is
            always `0.0` (the homed position), so no separate minimum is
            published. Carried even when `comms_ok` is false, since it is a
            configured fact rather than a live reading.
        zero_reference (ZeroReference): Whether the torso lift's drive origin is the one it was commissioned with.

            The LD2-RS keeps its zero in the absolute encoder, and every height this
            daemon accepts is measured from it. A drive that has lost that zero — a new
            drive, an EEPROM that did not survive, a swapped motor — still answers
            every register read: it simply calls some other place "0 m". A move to 0 m
            against such an origin drives the lift into its mechanical stop.

            So the origin is a published fact rather than an assumption, decided from
            two readings the drive itself supplies: `Pr0.15` (the absolute-encoder
            setting, which must be "multi-turn absolute enabled") and the current
            position, which on a commissioned drive has to lie inside the usable
            travel. See `d1fw_backends::slider::zero_reference`.
        abs_encoder_setting (int | None | Unset): The raw `Pr0.15` absolute-encoder setting the decision was made from
            (`1` = multi-turn absolute enabled, which is the only commissioned
            value), or `null` when the drive has not answered a reading of it.

            Published because `zero_reference: "unknown"` is otherwise a verdict
            with no evidence: this field and `height_m` are the two readings that
            produced it.
        advisory (Advisory | None | Unset):
        travel_limits (SliderTravelLimits | Unset): Every layer that constrains how far the torso lift may travel, and
            what
            they come to together.

            The layers exist because "how high can this lift go" has more than one
            answer and an operator needs to know WHICH one stopped a move. The model
            ceiling is what a D1's lift is; the unit calibration is what this
            particular machine's jig, cabling and mechanical build turned out to
            allow, measured on a bench and recorded in the daemon's state file.

            A layer only ever NARROWS: a unit calibration that claimed more travel
            than the model has is refused when it is recorded, so `effective` can
            never be wider than `model`.
        zero_verification (ZeroVerification | Unset): Whether a zero that was written has been proved to have survived a
            power
            cycle.

            Separate from [`ZeroReference`] on purpose, and additive to it: the two
            answer different questions and a consumer that only knows the older one
            keeps reading exactly what it read. `zero_reference` asks "can the drive
            vouch for its origin right now", which is decided from live readings.
            This asks "has the origin this daemon last wrote been read back after the
            power was removed", which no live reading can answer — it is a fact about
            history, so it lives in the daemon's state file.

            It matters because the LD2-RS reports its EEPROM save as taken (`0x5555`)
            before anything has proved the parameter is still there after power is
            removed, and an origin that lived in RAM only is precisely the failure
            `set_zero` exists to prevent.
    """

    alarm: bool
    alarm_code: int
    alarm_text: str
    comms_ok: bool
    height_m: float
    moving: bool
    travel_max_m: float
    zero_reference: ZeroReference
    abs_encoder_setting: int | None | Unset = UNSET
    advisory: Advisory | None | Unset = UNSET
    travel_limits: SliderTravelLimits | Unset = UNSET
    zero_verification: ZeroVerification | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        from ..models.advisory import Advisory

        alarm = self.alarm

        alarm_code = self.alarm_code

        alarm_text = self.alarm_text

        comms_ok = self.comms_ok

        height_m = self.height_m

        moving = self.moving

        travel_max_m = self.travel_max_m

        zero_reference = self.zero_reference.value

        abs_encoder_setting: int | None | Unset
        if isinstance(self.abs_encoder_setting, Unset):
            abs_encoder_setting = UNSET
        else:
            abs_encoder_setting = self.abs_encoder_setting

        advisory: dict[str, Any] | None | Unset
        if isinstance(self.advisory, Unset):
            advisory = UNSET
        elif isinstance(self.advisory, Advisory):
            advisory = self.advisory.to_dict()
        else:
            advisory = self.advisory

        travel_limits: dict[str, Any] | Unset = UNSET
        if not isinstance(self.travel_limits, Unset):
            travel_limits = self.travel_limits.to_dict()

        zero_verification: str | Unset = UNSET
        if not isinstance(self.zero_verification, Unset):
            zero_verification = self.zero_verification.value

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "alarm": alarm,
                "alarm_code": alarm_code,
                "alarm_text": alarm_text,
                "comms_ok": comms_ok,
                "height_m": height_m,
                "moving": moving,
                "travel_max_m": travel_max_m,
                "zero_reference": zero_reference,
            }
        )
        if abs_encoder_setting is not UNSET:
            field_dict["abs_encoder_setting"] = abs_encoder_setting
        if advisory is not UNSET:
            field_dict["advisory"] = advisory
        if travel_limits is not UNSET:
            field_dict["travel_limits"] = travel_limits
        if zero_verification is not UNSET:
            field_dict["zero_verification"] = zero_verification

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.advisory import Advisory
        from ..models.slider_travel_limits import SliderTravelLimits

        d = dict(src_dict)
        alarm = d.pop("alarm")

        alarm_code = d.pop("alarm_code")

        alarm_text = d.pop("alarm_text")

        comms_ok = d.pop("comms_ok")

        height_m = d.pop("height_m")

        moving = d.pop("moving")

        travel_max_m = d.pop("travel_max_m")

        zero_reference = ZeroReference(d.pop("zero_reference"))

        def _parse_abs_encoder_setting(data: object) -> int | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(int | None | Unset, data)

        abs_encoder_setting = _parse_abs_encoder_setting(
            d.pop("abs_encoder_setting", UNSET)
        )

        def _parse_advisory(data: object) -> Advisory | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            try:
                if not isinstance(data, dict):
                    raise TypeError()
                advisory_type_1 = Advisory.from_dict(data)

                return advisory_type_1
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            return cast(Advisory | None | Unset, data)

        advisory = _parse_advisory(d.pop("advisory", UNSET))

        _travel_limits = d.pop("travel_limits", UNSET)
        travel_limits: SliderTravelLimits | Unset
        if isinstance(_travel_limits, Unset):
            travel_limits = UNSET
        else:
            travel_limits = SliderTravelLimits.from_dict(_travel_limits)

        _zero_verification = d.pop("zero_verification", UNSET)
        zero_verification: ZeroVerification | Unset
        if isinstance(_zero_verification, Unset):
            zero_verification = UNSET
        else:
            zero_verification = ZeroVerification(_zero_verification)

        slider_state = cls(
            alarm=alarm,
            alarm_code=alarm_code,
            alarm_text=alarm_text,
            comms_ok=comms_ok,
            height_m=height_m,
            moving=moving,
            travel_max_m=travel_max_m,
            zero_reference=zero_reference,
            abs_encoder_setting=abs_encoder_setting,
            advisory=advisory,
            travel_limits=travel_limits,
            zero_verification=zero_verification,
        )

        slider_state.additional_properties = d
        return slider_state

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
