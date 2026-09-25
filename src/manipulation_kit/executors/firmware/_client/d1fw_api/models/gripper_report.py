from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.stroke_kind import StrokeKind
from ..types import UNSET, Unset

T = TypeVar("T", bound="GripperReport")


@_attrs_define
class GripperReport:
    """A gripper stroke report.

    Attributes:
        grip_preload_rad (float): Standing preload the hold keeps, in motor radians; zero when nothing
            is held.
        holding (bool): Whether the report indicates an object is held.
        jaw_rad (float): Measured jaw position in motor radians.
        kind (StrokeKind): The outcome category of a gripper stroke.
        torque_nm (float): Measured motor torque in newton-metres.
        coil_c (int | None | Unset): Coil temperature in °C from a live reading, when one was taken.
        fault_code (None | str | Unset): Which fault the motor is reporting, when `kind` is `fault`: one of
            `over_voltage`, `under_voltage`, `over_current`,
            `mos_over_temperature`, `coil_over_temperature`,
            `communication_lost`, `overload` or `unknown`.

            `null` when the gripper is not faulted, and also when the backend has
            no reading to decode one from. A bare `kind: fault` told an operator
            only that something was wrong; the cause is the difference between
            "let the coil cool" (`coil_over_temperature` does not clear in
            software at all) and "clear it and carry on".
        live (bool | Unset): Whether `jaw_rad` and `torque_nm` are a live reading of the idle motor
            (true) or the last stroke's telemetry (false, motor busy or silent).
        open_rad (float | None | Unset): Configured open ceiling in motor radians, when published by the backend.
        overload_released (bool | Unset): Whether the hold behind this report had to be opened PAST the measured
            jaws to bring its standing torque under the ceiling.

            The grip that is standing is not the grip that was asked for: the hold
            stands at zero preload, and `holding` says whether anything is still
            between the jaws. A caller carrying an object on the strength of a
            `grasp` needs to know the grasp was given up underneath it.
        target_closedness (float | None | Unset): The newest streamed target, as a closedness in `0.0..=1.0`, or `None`
            when nothing has been posted to the `target` lane since the last
            blocking stroke.

            This is what was ASKED for, not where the jaws are; `jaw_rad` is where
            they are.
        tracking (bool | Unset): Whether the `target` lane is driving the jaws toward
            `target_closedness` right now.

            False once the target has been reached and nothing new was posted, and
            false for every gripper driven only by the blocking verbs.
    """

    grip_preload_rad: float
    holding: bool
    jaw_rad: float
    kind: StrokeKind
    torque_nm: float
    coil_c: int | None | Unset = UNSET
    fault_code: None | str | Unset = UNSET
    live: bool | Unset = UNSET
    open_rad: float | None | Unset = UNSET
    overload_released: bool | Unset = UNSET
    target_closedness: float | None | Unset = UNSET
    tracking: bool | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        grip_preload_rad = self.grip_preload_rad

        holding = self.holding

        jaw_rad = self.jaw_rad

        kind = self.kind.value

        torque_nm = self.torque_nm

        coil_c: int | None | Unset
        if isinstance(self.coil_c, Unset):
            coil_c = UNSET
        else:
            coil_c = self.coil_c

        fault_code: None | str | Unset
        if isinstance(self.fault_code, Unset):
            fault_code = UNSET
        else:
            fault_code = self.fault_code

        live = self.live

        open_rad: float | None | Unset
        if isinstance(self.open_rad, Unset):
            open_rad = UNSET
        else:
            open_rad = self.open_rad

        overload_released = self.overload_released

        target_closedness: float | None | Unset
        if isinstance(self.target_closedness, Unset):
            target_closedness = UNSET
        else:
            target_closedness = self.target_closedness

        tracking = self.tracking

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "grip_preload_rad": grip_preload_rad,
                "holding": holding,
                "jaw_rad": jaw_rad,
                "kind": kind,
                "torque_nm": torque_nm,
            }
        )
        if coil_c is not UNSET:
            field_dict["coil_c"] = coil_c
        if fault_code is not UNSET:
            field_dict["fault_code"] = fault_code
        if live is not UNSET:
            field_dict["live"] = live
        if open_rad is not UNSET:
            field_dict["open_rad"] = open_rad
        if overload_released is not UNSET:
            field_dict["overload_released"] = overload_released
        if target_closedness is not UNSET:
            field_dict["target_closedness"] = target_closedness
        if tracking is not UNSET:
            field_dict["tracking"] = tracking

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        d = dict(src_dict)
        grip_preload_rad = d.pop("grip_preload_rad")

        holding = d.pop("holding")

        jaw_rad = d.pop("jaw_rad")

        kind = StrokeKind(d.pop("kind"))

        torque_nm = d.pop("torque_nm")

        def _parse_coil_c(data: object) -> int | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(int | None | Unset, data)

        coil_c = _parse_coil_c(d.pop("coil_c", UNSET))

        def _parse_fault_code(data: object) -> None | str | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(None | str | Unset, data)

        fault_code = _parse_fault_code(d.pop("fault_code", UNSET))

        live = d.pop("live", UNSET)

        def _parse_open_rad(data: object) -> float | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(float | None | Unset, data)

        open_rad = _parse_open_rad(d.pop("open_rad", UNSET))

        overload_released = d.pop("overload_released", UNSET)

        def _parse_target_closedness(data: object) -> float | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(float | None | Unset, data)

        target_closedness = _parse_target_closedness(d.pop("target_closedness", UNSET))

        tracking = d.pop("tracking", UNSET)

        gripper_report = cls(
            grip_preload_rad=grip_preload_rad,
            holding=holding,
            jaw_rad=jaw_rad,
            kind=kind,
            torque_nm=torque_nm,
            coil_c=coil_c,
            fault_code=fault_code,
            live=live,
            open_rad=open_rad,
            overload_released=overload_released,
            target_closedness=target_closedness,
            tracking=tracking,
        )

        gripper_report.additional_properties = d
        return gripper_report

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
