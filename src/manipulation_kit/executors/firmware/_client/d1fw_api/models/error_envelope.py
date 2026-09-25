from __future__ import annotations

from collections.abc import Mapping
from typing import (
    TYPE_CHECKING,
    Any,
    Literal,
    TypeVar,
    cast,
)

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.error_kind import ErrorKind
from ..types import UNSET, Unset

if TYPE_CHECKING:
    from ..models.advisory import Advisory


T = TypeVar("T", bound="ErrorEnvelope")


@_attrs_define
class ErrorEnvelope:
    """The failure envelope. `data` is always null; `message` always carries the daemon's own or the mobile base's error
    text; `kind` is always present and is the stable machine word to branch on; `code` is the mobile base's own numeric
    response code and is present only when the failure came from the base. Branch on `kind`, never on `message`.
    `advisory` is present only when the refusal knows the next step, and is absent otherwise.

        Attributes:
            data (None):
            kind (ErrorKind): A stable machine word for one failure, independent of its wording.

                Every error this daemon returns carries one of these, and a consumer
                branches on it rather than on the prose in `message`, which is the vendor's
                or the daemon's own sentence and is free to change. The values fall into
                two groups: the daemon's own conditions, and the mobile base's, which are
                the vendor response codes classified into what an operator can do about
                them.
            message (str):
            status (Literal['error']):
            advisory (Advisory | Unset): One suggestion about one device's current state.

                Additive everywhere it appears: absent when the state earns none.
            code (int | None | Unset): The mobile base's numeric response code, or null when the failure did not come from
                the base.
    """

    data: None
    kind: ErrorKind
    message: str
    status: Literal["error"]
    advisory: Advisory | Unset = UNSET
    code: int | None | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        data = self.data

        kind = self.kind.value

        message = self.message

        status = self.status

        advisory: dict[str, Any] | Unset = UNSET
        if not isinstance(self.advisory, Unset):
            advisory = self.advisory.to_dict()

        code: int | None | Unset
        if isinstance(self.code, Unset):
            code = UNSET
        else:
            code = self.code

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "data": data,
                "kind": kind,
                "message": message,
                "status": status,
            }
        )
        if advisory is not UNSET:
            field_dict["advisory"] = advisory
        if code is not UNSET:
            field_dict["code"] = code

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.advisory import Advisory

        d = dict(src_dict)
        data = d.pop("data")

        kind = ErrorKind(d.pop("kind"))

        message = d.pop("message")

        status = cast(Literal["error"], d.pop("status"))
        if status != "error":
            raise ValueError(f"status must match const 'error', got '{status}'")

        _advisory = d.pop("advisory", UNSET)
        advisory: Advisory | Unset
        if isinstance(_advisory, Unset):
            advisory = UNSET
        else:
            advisory = Advisory.from_dict(_advisory)

        def _parse_code(data: object) -> int | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(int | None | Unset, data)

        code = _parse_code(d.pop("code", UNSET))

        error_envelope = cls(
            data=data,
            kind=kind,
            message=message,
            status=status,
            advisory=advisory,
            code=code,
        )

        error_envelope.additional_properties = d
        return error_envelope

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
