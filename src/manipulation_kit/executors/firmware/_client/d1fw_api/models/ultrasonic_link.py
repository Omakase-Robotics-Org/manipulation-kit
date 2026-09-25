from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.ultrasonic_link_state import UltrasonicLinkState
from ..types import UNSET, Unset

T = TypeVar("T", bound="UltrasonicLink")


@_attrs_define
class UltrasonicLink:
    """How the daemon is currently reading the vendor's sensor push service.

    The service serves ONE client at a time, so how the daemon reads it is a
    decision with a cost to everything else that reads the base, and a
    consumer watching a distance change needs to know which of the two shapes
    is behind the numbers it is being shown. A reading taken by the
    [`UltrasonicLinkState::Cycle`] shape is up to a poll interval old by
    construction; one taken by [`UltrasonicLinkState::Held`] is as fresh as
    the base's own push rate.

        Attributes:
            detail (str): Empty unless something needs saying. For
                [`UltrasonicLinkState::Lost`], one sentence naming what was observed
                when the held connection ended.
            state (UltrasonicLinkState): The two shapes of read behind an [`UltrasonicLink`], plus the state of
                being between them.
            retry_in_ms (int | None | Unset): For [`UltrasonicLinkState::Lost`], roughly how long until the daemon
                tries to hold the connection again, in milliseconds. `null` otherwise.
    """

    detail: str
    state: UltrasonicLinkState
    retry_in_ms: int | None | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        detail = self.detail

        state = self.state.value

        retry_in_ms: int | None | Unset
        if isinstance(self.retry_in_ms, Unset):
            retry_in_ms = UNSET
        else:
            retry_in_ms = self.retry_in_ms

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "detail": detail,
                "state": state,
            }
        )
        if retry_in_ms is not UNSET:
            field_dict["retry_in_ms"] = retry_in_ms

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        detail = d.pop("detail")

        state = UltrasonicLinkState(d.pop("state"))

        def _parse_retry_in_ms(data: object) -> int | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(int | None | Unset, data)

        retry_in_ms = _parse_retry_in_ms(d.pop("retry_in_ms", UNSET))

        ultrasonic_link = cls(
            detail=detail,
            state=state,
            retry_in_ms=retry_in_ms,
        )

        ultrasonic_link.additional_properties = d
        return ultrasonic_link

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
