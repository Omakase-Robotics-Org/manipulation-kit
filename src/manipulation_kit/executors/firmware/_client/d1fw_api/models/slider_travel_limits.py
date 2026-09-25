from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..types import UNSET, Unset

if TYPE_CHECKING:
    from ..models.slider_travel_range import SliderTravelRange


T = TypeVar("T", bound="SliderTravelLimits")


@_attrs_define
class SliderTravelLimits:
    """Every layer that constrains how far the torso lift may travel, and what
    they come to together.

    The layers exist because "how high can this lift go" has more than one
    answer and an operator needs to know WHICH one stopped a move. The model
    ceiling is what a D1's lift is; the unit calibration is what this
    particular machine's jig, cabling and mechanical build turned out to
    allow, measured on a bench and recorded in the daemon's state file.

    A layer only ever NARROWS: a unit calibration that claimed more travel
    than the model has is refused when it is recorded, so `effective` can
    never be wider than `model`.

        Attributes:
            effective (SliderTravelRange): One closed interval of lift travel in metres.
            model (SliderTravelRange): One closed interval of lift travel in metres.
            unit (None | SliderTravelRange | Unset):
    """

    effective: SliderTravelRange
    model: SliderTravelRange
    unit: None | SliderTravelRange | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        from ..models.slider_travel_range import SliderTravelRange

        effective = self.effective.to_dict()

        model = self.model.to_dict()

        unit: dict[str, Any] | None | Unset
        if isinstance(self.unit, Unset):
            unit = UNSET
        elif isinstance(self.unit, SliderTravelRange):
            unit = self.unit.to_dict()
        else:
            unit = self.unit

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "effective": effective,
                "model": model,
            }
        )
        if unit is not UNSET:
            field_dict["unit"] = unit

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.slider_travel_range import SliderTravelRange

        d = dict(src_dict)
        effective = SliderTravelRange.from_dict(d.pop("effective"))

        model = SliderTravelRange.from_dict(d.pop("model"))

        def _parse_unit(data: object) -> None | SliderTravelRange | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            try:
                if not isinstance(data, dict):
                    raise TypeError()
                unit_type_1 = SliderTravelRange.from_dict(data)

                return unit_type_1
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            return cast(None | SliderTravelRange | Unset, data)

        unit = _parse_unit(d.pop("unit", UNSET))

        slider_travel_limits = cls(
            effective=effective,
            model=model,
            unit=unit,
        )

        slider_travel_limits.additional_properties = d
        return slider_travel_limits

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
