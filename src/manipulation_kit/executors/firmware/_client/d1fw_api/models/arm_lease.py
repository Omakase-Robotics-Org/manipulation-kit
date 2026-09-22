from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.lease_class import LeaseClass
from ..types import UNSET, Unset

T = TypeVar("T", bound="ArmLease")


@_attrs_define
class ArmLease:
    """The published view of the lease, as `POST /v1/arm/lease`,
    `GET /v1/arm/lease` and the `arm.lease` slot of `GET /v1/state` return it.

        Attributes:
            class_ (LeaseClass): How privileged one lease request is, and therefore what it may displace:
                `ambient` (rank 10) < `policy` (50) < `operator` (90).

                A request of a strictly higher class than the current holder preempts it;
                an equal class is first-come and a lower one is refused. Three named
                classes rather than a free integer, because a rank is only useful if two
                consumers written by two people mean the same thing by it. The numeric
                ranks are published for consumers whose own arbiter works in numbers —
                omakase-core's does — but the wire form is always the name.
            epoch (int): Monotonic grant counter, incremented on every FRESH grant and left
                alone by a heartbeat renewal.

                This is what tells a consumer that reads `holder == "me"` whether it
                is the same grant it had a moment ago: an unchanged `epoch` means it
                never lost the arms, while a higher one means it was displaced and
                later granted them again — and therefore that whatever it believed
                about the arms' pose, mode or tool registration is stale.
            expires_in_s (int): Whole seconds until the lease expires, rounded up. Once this reaches
                zero the lease is gone: the next caller takes it without asking.
            holder (str): The self-declared name of the consumer holding the arms.
            ttl_s (int): The TTL this lease was granted with, in seconds.
            preempted_from (None | str | Unset): The holder this lease was taken away from, when it was taken by
                preemption rather than granted freely. `null` for an ordinary grant.

                It survives renewals, because it describes how this grant came about
                and that does not change when the holder heartbeats.
    """

    class_: LeaseClass
    epoch: int
    expires_in_s: int
    holder: str
    ttl_s: int
    preempted_from: None | str | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        class_ = self.class_.value

        epoch = self.epoch

        expires_in_s = self.expires_in_s

        holder = self.holder

        ttl_s = self.ttl_s

        preempted_from: None | str | Unset
        if isinstance(self.preempted_from, Unset):
            preempted_from = UNSET
        else:
            preempted_from = self.preempted_from

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "class": class_,
                "epoch": epoch,
                "expires_in_s": expires_in_s,
                "holder": holder,
                "ttl_s": ttl_s,
            }
        )
        if preempted_from is not UNSET:
            field_dict["preempted_from"] = preempted_from

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        class_ = LeaseClass(d.pop("class"))

        epoch = d.pop("epoch")

        expires_in_s = d.pop("expires_in_s")

        holder = d.pop("holder")

        ttl_s = d.pop("ttl_s")

        def _parse_preempted_from(data: object) -> None | str | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(None | str | Unset, data)

        preempted_from = _parse_preempted_from(d.pop("preempted_from", UNSET))

        arm_lease = cls(
            class_=class_,
            epoch=epoch,
            expires_in_s=expires_in_s,
            holder=holder,
            ttl_s=ttl_s,
            preempted_from=preempted_from,
        )

        arm_lease.additional_properties = d
        return arm_lease

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
