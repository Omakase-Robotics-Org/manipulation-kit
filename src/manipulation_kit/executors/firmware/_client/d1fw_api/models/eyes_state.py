from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..types import UNSET, Unset

if TYPE_CHECKING:
    from ..models.eyes_playback_state import EyesPlaybackState


T = TypeVar("T", bound="EyesState")


@_attrs_define
class EyesState:
    """Eye controller feedback.

    Attributes:
        initialized (bool): Whether the controller has completed its initialization sequence.
        mode (str): Current controller mode, such as `manual` or `off`.
        last_effect (None | str | Unset): Last effect name reported by the controller, if any.
        playing (EyesPlaybackState | None | Unset):
    """

    initialized: bool
    mode: str
    last_effect: None | str | Unset = UNSET
    playing: EyesPlaybackState | None | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        from ..models.eyes_playback_state import EyesPlaybackState

        initialized = self.initialized

        mode = self.mode

        last_effect: None | str | Unset
        if isinstance(self.last_effect, Unset):
            last_effect = UNSET
        else:
            last_effect = self.last_effect

        playing: dict[str, Any] | None | Unset
        if isinstance(self.playing, Unset):
            playing = UNSET
        elif isinstance(self.playing, EyesPlaybackState):
            playing = self.playing.to_dict()
        else:
            playing = self.playing

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "initialized": initialized,
                "mode": mode,
            }
        )
        if last_effect is not UNSET:
            field_dict["last_effect"] = last_effect
        if playing is not UNSET:
            field_dict["playing"] = playing

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.eyes_playback_state import EyesPlaybackState

        d = dict(src_dict)
        initialized = d.pop("initialized")

        mode = d.pop("mode")

        def _parse_last_effect(data: object) -> None | str | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(None | str | Unset, data)

        last_effect = _parse_last_effect(d.pop("last_effect", UNSET))

        def _parse_playing(data: object) -> EyesPlaybackState | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            try:
                if not isinstance(data, dict):
                    raise TypeError()
                playing_type_1 = EyesPlaybackState.from_dict(data)

                return playing_type_1
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            return cast(EyesPlaybackState | None | Unset, data)

        playing = _parse_playing(d.pop("playing", UNSET))

        eyes_state = cls(
            initialized=initialized,
            mode=mode,
            last_effect=last_effect,
            playing=playing,
        )

        eyes_state.additional_properties = d
        return eyes_state

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
