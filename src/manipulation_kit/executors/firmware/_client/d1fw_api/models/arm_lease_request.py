from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.lease_class import LeaseClass
from ..types import UNSET, Unset

T = TypeVar("T", bound="ArmLeaseRequest")


@_attrs_define
class ArmLeaseRequest:
    """`POST /v1/arm/lease`: take, renew, or preempt the arm lease.

    Attributes:
        holder (str): The consumer's own name, echoed back in every refusal so an operator
            can see who to ask. Free-form, but use something a human can act on:
            `dx-vr-teleop`, `omakaseos`, `console`.
        class_ (LeaseClass | None | Unset):
        ttl_s (int | None | Unset): How long the lease should live, in seconds. Omitted means
            [`DEFAULT_LEASE_TTL_S`]; the accepted range is `1..=600`. Re-POST with
            the same holder to heartbeat.
    """

    holder: str
    class_: LeaseClass | None | Unset = UNSET
    ttl_s: int | None | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        holder = self.holder

        class_: None | str | Unset
        if isinstance(self.class_, Unset):
            class_ = UNSET
        elif isinstance(self.class_, LeaseClass):
            class_ = self.class_.value
        else:
            class_ = self.class_

        ttl_s: int | None | Unset
        if isinstance(self.ttl_s, Unset):
            ttl_s = UNSET
        else:
            ttl_s = self.ttl_s

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "holder": holder,
            }
        )
        if class_ is not UNSET:
            field_dict["class"] = class_
        if ttl_s is not UNSET:
            field_dict["ttl_s"] = ttl_s

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        holder = d.pop("holder")

        def _parse_class_(data: object) -> LeaseClass | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            try:
                if not isinstance(data, str):
                    raise TypeError()
                class_type_1 = LeaseClass(data)

                return class_type_1
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            return cast(LeaseClass | None | Unset, data)

        class_ = _parse_class_(d.pop("class", UNSET))

        def _parse_ttl_s(data: object) -> int | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(int | None | Unset, data)

        ttl_s = _parse_ttl_s(d.pop("ttl_s", UNSET))

        arm_lease_request = cls(
            holder=holder,
            class_=class_,
            ttl_s=ttl_s,
        )

        arm_lease_request.additional_properties = d
        return arm_lease_request

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
