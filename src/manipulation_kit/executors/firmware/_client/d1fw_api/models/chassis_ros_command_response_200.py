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

if TYPE_CHECKING:
    from ..models.vendor_ros_command_report import VendorRosCommandReport


T = TypeVar("T", bound="ChassisRosCommandResponse200")


@_attrs_define
class ChassisRosCommandResponse200:
    """
    Attributes:
        data (VendorRosCommandReport): Receipt from one fixed ROS service or pose publish attempt; never auto-retry.

            For command=pose_reset the only outcomes are transport_sent and outcome_unknown,
            vendor_success is always present and null, physical_completion is unknown and
            mapping_observation is absent. transport_sent means only local WebSocket write
            completion; no ROS subscriber acceptance, reset, convergence or physical completion
            is established. An acknowledged/refused pose report is a contract violation, not
            evidence of success/rejection; treat delivery as unknown. Service commands retain
            acknowledged/refused/outcome_unknown and their matching service success flag.

            For pose_reset, a complete authentic daemon error envelope (status=error, data=null,
            kind and message) represents rejection/setup failure before the pose publish attempt.
            Setup may advertise the fixed topic but carries no pose. Once publish send begins,
            failure/timeout returns an outcome_unknown report instead of an ordinary error.
            A missing, malformed, proxy-generated or network response does not establish this
            prepublish guarantee. Request IDs are correlation only; reread and review, never retry.
        message (None): Always null on success.
        status (Literal['ok']):
    """

    data: VendorRosCommandReport
    message: None
    status: Literal["ok"]
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        data = self.data.to_dict()

        message = self.message

        status = self.status

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "data": data,
                "message": message,
                "status": status,
            }
        )

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.vendor_ros_command_report import (
            VendorRosCommandReport,
        )

        d = dict(src_dict)
        data = VendorRosCommandReport.from_dict(d.pop("data"))

        message = d.pop("message")

        status = cast(Literal["ok"], d.pop("status"))
        if status != "ok":
            raise ValueError(f"status must match const 'ok', got '{status}'")

        chassis_ros_command_response_200 = cls(
            data=data,
            message=message,
            status=status,
        )

        chassis_ros_command_response_200.additional_properties = d
        return chassis_ros_command_response_200

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
