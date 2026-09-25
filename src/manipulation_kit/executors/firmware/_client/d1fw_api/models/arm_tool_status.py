from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.arm_side import ArmSide
from ..models.arm_tool_source import ArmToolSource
from ..types import UNSET, Unset

if TYPE_CHECKING:
    from ..models.tool_config import ToolConfig


T = TypeVar("T", bound="ArmToolStatus")


@_attrs_define
class ArmToolStatus:
    """What the daemon has registered on one arm, and where it came from.

    The arm controller does not report its registered tool in feedback, so
    this is the daemon's own record of what it last sent. It is the answer to
    "is this arm compensating for the gripper it is actually wearing?", which
    is otherwise only visible as a sagging wrist.

        Attributes:
            side (ArmSide): Selects one of the two physical arms.
            source (ArmToolSource): Where the tool registration one arm is holding came from.
            model (None | str | Unset): The `"<maker>/<model>"` end-effector id, when the registration came
                from the daemon's catalog. A client registration carries raw numbers
                and no model name, so this is `null` for one.
            tool (None | ToolConfig | Unset):
    """

    side: ArmSide
    source: ArmToolSource
    model: None | str | Unset = UNSET
    tool: None | ToolConfig | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        from ..models.tool_config import ToolConfig

        side = self.side.value

        source = self.source.value

        model: None | str | Unset
        if isinstance(self.model, Unset):
            model = UNSET
        else:
            model = self.model

        tool: dict[str, Any] | None | Unset
        if isinstance(self.tool, Unset):
            tool = UNSET
        elif isinstance(self.tool, ToolConfig):
            tool = self.tool.to_dict()
        else:
            tool = self.tool

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "side": side,
                "source": source,
            }
        )
        if model is not UNSET:
            field_dict["model"] = model
        if tool is not UNSET:
            field_dict["tool"] = tool

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.tool_config import ToolConfig

        d = dict(src_dict)
        side = ArmSide(d.pop("side"))

        source = ArmToolSource(d.pop("source"))

        def _parse_model(data: object) -> None | str | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(None | str | Unset, data)

        model = _parse_model(d.pop("model", UNSET))

        def _parse_tool(data: object) -> None | ToolConfig | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            try:
                if not isinstance(data, dict):
                    raise TypeError()
                tool_type_1 = ToolConfig.from_dict(data)

                return tool_type_1
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            return cast(None | ToolConfig | Unset, data)

        tool = _parse_tool(d.pop("tool", UNSET))

        arm_tool_status = cls(
            side=side,
            source=source,
            model=model,
            tool=tool,
        )

        arm_tool_status.additional_properties = d
        return arm_tool_status

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
