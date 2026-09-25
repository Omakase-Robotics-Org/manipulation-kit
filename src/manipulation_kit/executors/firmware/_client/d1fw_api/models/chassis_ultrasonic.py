from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..types import UNSET, Unset

if TYPE_CHECKING:
    from ..models.ultrasonic_channel import UltrasonicChannel
    from ..models.ultrasonic_link import UltrasonicLink


T = TypeVar("T", bound="ChassisUltrasonic")


@_attrs_define
class ChassisUltrasonic:
    """The mobile base's ultrasonic proximity ring, as the daemon last read it.

    This is a HARDWARE-CHECK reading. The daemon does not brake, stop or
    re-route the base on it and nothing here is a safety interlock; it exists
    so an operator can see whether the ring is working and what it sees.

    The ring is published only by the vendor's TCP sensor push service, which
    one mobile-base firmware generation serves and the other does not, so
    "this base cannot report sonar" is a normal answer and is reported as
    [`ChassisUltrasonic::supported`] `false` with a stated
    [`ChassisUltrasonic::reason`] rather than as an error or as an empty
    reading that looks like a silent ring.

        Attributes:
            channels (list[UltrasonicChannel]): One entry per channel the daemon has a reading for, ordered by index.
                Empty whenever there is no reading.
            reason (str): Empty when a reading was obtained. Otherwise one sentence saying why
                there is none — an unsupported generation, an unplaceable one, or the
                transport failure from the attempted read.
            source (str): Where the reading came from: `sensor-stream`, or `none` when there is
                no reading.
            supported (bool): Whether this base can report an ultrasonic ring at all: whether its
                mobile-base firmware generation serves the sensor push service.

                `false` is a statement about the base, not about this read. A base
                that is supported but whose service could not be reached has
                `supported` `true`, a `source` of `none`, and the transport failure in
                `reason`.
            age_ms (int | None | Unset): How long ago the newest frame behind this reading was read, in
                milliseconds, or `null` when there is no reading.
            frame_id (None | str | Unset): The vendor frame's source identifier; the D1 mobile base sends
                `sonar_array`. `null` when there is no reading.
            link (UltrasonicLink | Unset): How the daemon is currently reading the vendor's sensor push service.

                The service serves ONE client at a time, so how the daemon reads it is a
                decision with a cost to everything else that reads the base, and a
                consumer watching a distance change needs to know which of the two shapes
                is behind the numbers it is being shown. A reading taken by the
                [`UltrasonicLinkState::Cycle`] shape is up to a poll interval old by
                construction; one taken by [`UltrasonicLinkState::Held`] is as fresh as
                the base's own push rate.
            seq (int | None | Unset): The vendor frame's global sequence number, or `null` when there is no
                reading.
            stamp_s (float | None | Unset): The vendor frame's timestamp in seconds, or `null` when there is no
                reading.
    """

    channels: list[UltrasonicChannel]
    reason: str
    source: str
    supported: bool
    age_ms: int | None | Unset = UNSET
    frame_id: None | str | Unset = UNSET
    link: UltrasonicLink | Unset = UNSET
    seq: int | None | Unset = UNSET
    stamp_s: float | None | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        channels = []
        for channels_item_data in self.channels:
            channels_item = channels_item_data.to_dict()
            channels.append(channels_item)

        reason = self.reason

        source = self.source

        supported = self.supported

        age_ms: int | None | Unset
        if isinstance(self.age_ms, Unset):
            age_ms = UNSET
        else:
            age_ms = self.age_ms

        frame_id: None | str | Unset
        if isinstance(self.frame_id, Unset):
            frame_id = UNSET
        else:
            frame_id = self.frame_id

        link: dict[str, Any] | Unset = UNSET
        if not isinstance(self.link, Unset):
            link = self.link.to_dict()

        seq: int | None | Unset
        if isinstance(self.seq, Unset):
            seq = UNSET
        else:
            seq = self.seq

        stamp_s: float | None | Unset
        if isinstance(self.stamp_s, Unset):
            stamp_s = UNSET
        else:
            stamp_s = self.stamp_s

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "channels": channels,
                "reason": reason,
                "source": source,
                "supported": supported,
            }
        )
        if age_ms is not UNSET:
            field_dict["age_ms"] = age_ms
        if frame_id is not UNSET:
            field_dict["frame_id"] = frame_id
        if link is not UNSET:
            field_dict["link"] = link
        if seq is not UNSET:
            field_dict["seq"] = seq
        if stamp_s is not UNSET:
            field_dict["stamp_s"] = stamp_s

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.ultrasonic_channel import UltrasonicChannel
        from ..models.ultrasonic_link import UltrasonicLink

        d = dict(src_dict)
        channels = []
        _channels = d.pop("channels")
        for channels_item_data in _channels:
            channels_item = UltrasonicChannel.from_dict(channels_item_data)

            channels.append(channels_item)

        reason = d.pop("reason")

        source = d.pop("source")

        supported = d.pop("supported")

        def _parse_age_ms(data: object) -> int | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(int | None | Unset, data)

        age_ms = _parse_age_ms(d.pop("age_ms", UNSET))

        def _parse_frame_id(data: object) -> None | str | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(None | str | Unset, data)

        frame_id = _parse_frame_id(d.pop("frame_id", UNSET))

        _link = d.pop("link", UNSET)
        link: UltrasonicLink | Unset
        if isinstance(_link, Unset):
            link = UNSET
        else:
            link = UltrasonicLink.from_dict(_link)

        def _parse_seq(data: object) -> int | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(int | None | Unset, data)

        seq = _parse_seq(d.pop("seq", UNSET))

        def _parse_stamp_s(data: object) -> float | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(float | None | Unset, data)

        stamp_s = _parse_stamp_s(d.pop("stamp_s", UNSET))

        chassis_ultrasonic = cls(
            channels=channels,
            reason=reason,
            source=source,
            supported=supported,
            age_ms=age_ms,
            frame_id=frame_id,
            link=link,
            seq=seq,
            stamp_s=stamp_s,
        )

        chassis_ultrasonic.additional_properties = d
        return chassis_ultrasonic

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
