from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.slave_al_state_type_0 import SlaveAlStateType0
from ..models.slave_al_state_type_1 import SlaveAlStateType1
from ..models.slave_al_state_type_2 import SlaveAlStateType2
from ..models.slave_al_state_type_3 import SlaveAlStateType3
from ..models.slave_al_state_type_4 import SlaveAlStateType4

if TYPE_CHECKING:
    from ..models.slave_al_state_type_5 import SlaveAlStateType5


T = TypeVar("T", bound="EtherCatSlave")


@_attrs_define
class EtherCatSlave:
    """One EtherCAT slave on a master's ring (from `ethercat slaves`).

    Attributes:
        alias (str): Alias/address token as printed by the tool (e.g. `0:3`).
        name (str): Product name (e.g. `CoolDrive JMDT` or `app`).
        position (int): Ring position (0-based).
        state (SlaveAlStateType0 | SlaveAlStateType1 | SlaveAlStateType2 | SlaveAlStateType3 | SlaveAlStateType4 |
            SlaveAlStateType5): EtherCAT slave application-layer (AL) state.

            The recognized values are the IgH EtherLab AL states; any other token the
            tool emits is preserved through [`SlaveAlState::Other`] rather than dropped.
    """

    alias: str
    name: str
    position: int
    state: (
        SlaveAlStateType0
        | SlaveAlStateType1
        | SlaveAlStateType2
        | SlaveAlStateType3
        | SlaveAlStateType4
        | SlaveAlStateType5
    )
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        alias = self.alias

        name = self.name

        position = self.position

        state: dict[str, Any] | str
        if (
            isinstance(self.state, SlaveAlStateType0)
            or isinstance(self.state, SlaveAlStateType1)
            or isinstance(self.state, SlaveAlStateType2)
            or isinstance(self.state, SlaveAlStateType3)
            or isinstance(self.state, SlaveAlStateType4)
        ):
            state = self.state.value
        else:
            state = self.state.to_dict()

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "alias": alias,
                "name": name,
                "position": position,
                "state": state,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.slave_al_state_type_5 import SlaveAlStateType5

        d = dict(src_dict)
        alias = d.pop("alias")

        name = d.pop("name")

        position = d.pop("position")

        def _parse_state(
            data: object,
        ) -> (
            SlaveAlStateType0
            | SlaveAlStateType1
            | SlaveAlStateType2
            | SlaveAlStateType3
            | SlaveAlStateType4
            | SlaveAlStateType5
        ):
            try:
                if not isinstance(data, str):
                    raise TypeError()
                componentsschemas_slave_al_state_type_0 = SlaveAlStateType0(data)

                return componentsschemas_slave_al_state_type_0
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            try:
                if not isinstance(data, str):
                    raise TypeError()
                componentsschemas_slave_al_state_type_1 = SlaveAlStateType1(data)

                return componentsschemas_slave_al_state_type_1
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            try:
                if not isinstance(data, str):
                    raise TypeError()
                componentsschemas_slave_al_state_type_2 = SlaveAlStateType2(data)

                return componentsschemas_slave_al_state_type_2
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            try:
                if not isinstance(data, str):
                    raise TypeError()
                componentsschemas_slave_al_state_type_3 = SlaveAlStateType3(data)

                return componentsschemas_slave_al_state_type_3
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            try:
                if not isinstance(data, str):
                    raise TypeError()
                componentsschemas_slave_al_state_type_4 = SlaveAlStateType4(data)

                return componentsschemas_slave_al_state_type_4
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            if not isinstance(data, dict):
                raise TypeError()
            componentsschemas_slave_al_state_type_5 = SlaveAlStateType5.from_dict(data)

            return componentsschemas_slave_al_state_type_5

        state = _parse_state(d.pop("state"))

        ether_cat_slave = cls(
            alias=alias,
            name=name,
            position=position,
            state=state,
        )

        ether_cat_slave.additional_properties = d
        return ether_cat_slave

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
