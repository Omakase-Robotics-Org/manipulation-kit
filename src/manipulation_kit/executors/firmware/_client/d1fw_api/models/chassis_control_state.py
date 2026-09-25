from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeVar, cast

from attrs import define as _attrs_define
from attrs import field as _attrs_field
from typing_extensions import Self

from ..models.motor_source import MotorSource
from ..models.motor_state import MotorState
from ..models.navigation_state import NavigationState
from ..models.work_mode_type_0 import WorkModeType0
from ..models.work_mode_type_1 import WorkModeType1
from ..models.work_mode_type_2 import WorkModeType2
from ..models.work_mode_type_3 import WorkModeType3
from ..models.work_mode_type_4 import WorkModeType4
from ..types import UNSET, Unset

if TYPE_CHECKING:
    from ..models.chassis_transition import ChassisTransition
    from ..models.hand_drive_blocker import HandDriveBlocker
    from ..models.work_mode_type_5 import WorkModeType5


T = TypeVar("T", bound="ChassisControlState")


@_attrs_define
class ChassisControlState:
    """Everything the daemon knows about WHICH CONTROLLER may drive the mobile
    base: the mode it is in, what that mode permits, and which control verbs
    are legal from it.

    The mobile base has exactly one control mode at a time and every motion
    path is gated on it, but the vendor publishes that only as a bare
    `workMode` digit and enforces it by IGNORING commands: outside its
    remote-control mode the base answers a jog with HTTP 200 and does not
    move. This object is the daemon's answer to "what state is the base in,
    what can I do from here, and why not" in one read, so that an operator
    interface never has to infer it from a digit or from a command that
    silently did nothing.

    The five vendor modes are the vendor's own, taken from its web
    application's own translation table: `0` navigation closed, `1`
    navigation, `2` automatic charging, `3` mapping, `4` remote control.

        Attributes:
            bumper (bool): Whether a bumper input is latched: the same fact as
                [`ChassisState::bumper_pressed`], which is the OR of the vendor's
                `stripStatus` and `stopStatus` fields.
            can_hand_drive (bool): Whether the reported control preconditions permit sending a jog or
                nonzero velocity command, before the separate software stop gate.

                True only when the base is in remote-control mode, is not reported in standby,
                has its emergency switch released, and has not had its motors released
                by this daemon. [`MotorState::Unknown`] does not block: nothing on the
                base reports the motor state, so refusing on it would make hand-drive
                permanently unavailable on a daemon that has just started. Unknown
                standby also does not block. A true value is not proof of physical
                movement: motor command memory can be stale, and the base's bumper
                interlock can stop motion without blocking command transmission.
                The daemon's software stop latch can still refuse the request.
            emergency_switch (bool): Whether the base's physical emergency-stop input reads as pressed.

                The same fact as [`ChassisState::emergency_stop`], repeated here so
                that `GET /v1/chassis/control` answers every precondition of driving
                the base without a second read.
            hand_drive_blockers (list[HandDriveBlocker]): Every precondition of [`Self::can_hand_drive`] that is not met,
                each
                with the sentence that says what clears it. Empty when
                `can_hand_drive` is true.
            local_goal_id (None | str): Which route-network point the base is currently driving to, as the
                vendor's `localGoalId` names it, or `null` when it is driving to none.

                This is the base's OWN answer about its own trip, and it is the only
                one that survives this daemon restarting: the daemon's `navigate`
                handles (`NavId`) are process-local, so after a restart -- or for a
                trip somebody started from the base's own web interface -- this is the
                only identification of what is running.

                `"0"` is normalised to `null` here. It is the vendor's idle sentinel,
                measured on D1 #1 while the base was doing nothing, and it cannot
                collide with a real point: route-network point identifiers are
                zero-padded to four digits (`"0001"`, `"0007"`), so a bare `"0"` is
                never one of them. An empty string is normalised the same way. The
                unnormalised value is in [`Self::local_goal_id_raw`], so the
                normalisation destroys nothing.
            local_goal_id_raw (None | str): The vendor's `localGoalId` exactly as it arrived, or `null` when the
                base sent the field empty or did not send it at all.
            mode (WorkModeType0 | WorkModeType1 | WorkModeType2 | WorkModeType3 | WorkModeType4 | WorkModeType5): The
                decoded chassis work mode.
            motors (MotorState): Whether the mobile base's drive motors are engaged, as far as this daemon
                can tell.

                The vendor's `lockCtrl` endpoint writes this and NOTHING reads it back:
                `getRobotInfo` carries no field for it and the vendor's own web
                application never displays one. So this is the daemon's memory of its own
                last successful write, and [`ChassisControlState::motors_source`] says
                whether there was one.
            motors_source (MotorSource): Where [`ChassisControlState::motors`] came from.
            navigation (NavigationState): The mobile base's autonomous-navigation state, decoded from the vendor's
                `autoStatus` field.

                The vendor's own dictionary for that field, as its web application's
                status page renders it and as the vendor SDK's `monitor/codes.py` spells
                it, is `0` navigation not started, `1` localizing, `2` ready/arrived, `3`
                moving, `4` path blocked, `5` paused, `6` the goal point is occupied, `7`
                localization lost, `8` sensor data lost.

                `autoStatus` is a NAVIGATION field, and the base does not clear it when it
                leaves navigation mode: a base sitting in the vendor's remote-control mode
                can still be reporting the `3` it was reporting when its last trip was
                interrupted. Reading it without checking the work mode first is therefore
                how a consumer convinces itself a stationary base is driving, so this is
                [`Off`](Self::Off) whenever [`ChassisControlState::mode`] is not
                [`WorkMode::Navigation`], and the untouched field is still on the wire as
                [`ChassisControlState::auto_status_raw`].
            remote_control (bool): Whether that mode is the vendor's remote-control mode (`workMode`
                `4`), which is the only mode in which the base acts on a jog or a
                velocity command.
            transitions (list[ChassisTransition]): Which control verbs are legal from this state, and why the others are
                not.

                This describes the MOBILE BASE's own mode machine and nothing else.
                The daemon's soft-kill latch is a separate gate that can still refuse
                an `allowed: true` verb with the `kill_latched` failure kind; read
                `/v1/state`'s `soft_killed` for that.
            auto_status_raw (None | str | Unset): The vendor's `autoStatus` string exactly as it arrived, whatever mode
                the base is in, or `null` when the base reported no such field.
            standby (bool | None | Unset): Whether the base is in its standby (sleep) state: `true` for the
                vendor's documented `standbyStatus` 1, `false` for the documented 0,
                and `null` when the base reported no such field or a value the vendor
                documents nowhere.

                A standby base stops acting on remote-control commands. It is woken
                over the base's own ROS layer (`SleepCtrl`), which has no HTTP verb,
                so this daemon reports the state and cannot change it.
            standby_raw (None | str | Unset): The vendor's `standbyStatus` string exactly as it arrived, or `null`
                when the base reported no such field.
            stop_status_raw (None | str | Unset): The vendor's `stopStatus` string exactly as it arrived, or `null`.

                The vendor publishes NO dictionary for this field and its own programs
                read it three incompatible ways, so the daemon folds any non-zero
                value into [`Self::bumper`] and publishes the raw value here rather
                than inventing a meaning for it.
            strip_status_raw (None | str | Unset): The vendor's `stripStatus` string exactly as it arrived, or `null`.
                This is the anti-collision bumper strip, `0` not triggered and `1`
                triggered, cleared by `POST /v1/chassis/remove_strip`.
    """

    bumper: bool
    can_hand_drive: bool
    emergency_switch: bool
    hand_drive_blockers: list[HandDriveBlocker]
    local_goal_id: None | str
    local_goal_id_raw: None | str
    mode: (
        WorkModeType0
        | WorkModeType1
        | WorkModeType2
        | WorkModeType3
        | WorkModeType4
        | WorkModeType5
    )
    motors: MotorState
    motors_source: MotorSource
    navigation: NavigationState
    remote_control: bool
    transitions: list[ChassisTransition]
    auto_status_raw: None | str | Unset = UNSET
    standby: bool | None | Unset = UNSET
    standby_raw: None | str | Unset = UNSET
    stop_status_raw: None | str | Unset = UNSET
    strip_status_raw: None | str | Unset = UNSET
    additional_properties: dict[str, Any] = _attrs_field(init=False, factory=dict)

    def to_dict(self) -> dict[str, Any]:
        bumper = self.bumper

        can_hand_drive = self.can_hand_drive

        emergency_switch = self.emergency_switch

        hand_drive_blockers = []
        for hand_drive_blockers_item_data in self.hand_drive_blockers:
            hand_drive_blockers_item = hand_drive_blockers_item_data.to_dict()
            hand_drive_blockers.append(hand_drive_blockers_item)

        local_goal_id: None | str
        local_goal_id = self.local_goal_id

        local_goal_id_raw: None | str
        local_goal_id_raw = self.local_goal_id_raw

        mode: dict[str, Any] | str
        if (
            isinstance(self.mode, WorkModeType0)
            or isinstance(self.mode, WorkModeType1)
            or isinstance(self.mode, WorkModeType2)
            or isinstance(self.mode, WorkModeType3)
            or isinstance(self.mode, WorkModeType4)
        ):
            mode = self.mode.value
        else:
            mode = self.mode.to_dict()

        motors = self.motors.value

        motors_source = self.motors_source.value

        navigation = self.navigation.value

        remote_control = self.remote_control

        transitions = []
        for transitions_item_data in self.transitions:
            transitions_item = transitions_item_data.to_dict()
            transitions.append(transitions_item)

        auto_status_raw: None | str | Unset
        if isinstance(self.auto_status_raw, Unset):
            auto_status_raw = UNSET
        else:
            auto_status_raw = self.auto_status_raw

        standby: bool | None | Unset
        if isinstance(self.standby, Unset):
            standby = UNSET
        else:
            standby = self.standby

        standby_raw: None | str | Unset
        if isinstance(self.standby_raw, Unset):
            standby_raw = UNSET
        else:
            standby_raw = self.standby_raw

        stop_status_raw: None | str | Unset
        if isinstance(self.stop_status_raw, Unset):
            stop_status_raw = UNSET
        else:
            stop_status_raw = self.stop_status_raw

        strip_status_raw: None | str | Unset
        if isinstance(self.strip_status_raw, Unset):
            strip_status_raw = UNSET
        else:
            strip_status_raw = self.strip_status_raw

        field_dict: dict[str, Any] = {}
        field_dict.update(self.additional_properties)
        field_dict.update(
            {
                "bumper": bumper,
                "can_hand_drive": can_hand_drive,
                "emergency_switch": emergency_switch,
                "hand_drive_blockers": hand_drive_blockers,
                "local_goal_id": local_goal_id,
                "local_goal_id_raw": local_goal_id_raw,
                "mode": mode,
                "motors": motors,
                "motors_source": motors_source,
                "navigation": navigation,
                "remote_control": remote_control,
                "transitions": transitions,
            }
        )
        if auto_status_raw is not UNSET:
            field_dict["auto_status_raw"] = auto_status_raw
        if standby is not UNSET:
            field_dict["standby"] = standby
        if standby_raw is not UNSET:
            field_dict["standby_raw"] = standby_raw
        if stop_status_raw is not UNSET:
            field_dict["stop_status_raw"] = stop_status_raw
        if strip_status_raw is not UNSET:
            field_dict["strip_status_raw"] = strip_status_raw

        return field_dict

    @classmethod
    def from_dict(cls, src_dict: Mapping[str, Any]) -> Self:
        from ..models.chassis_transition import ChassisTransition
        from ..models.hand_drive_blocker import HandDriveBlocker
        from ..models.work_mode_type_5 import WorkModeType5

        d = dict(src_dict)
        bumper = d.pop("bumper")

        can_hand_drive = d.pop("can_hand_drive")

        emergency_switch = d.pop("emergency_switch")

        hand_drive_blockers = []
        _hand_drive_blockers = d.pop("hand_drive_blockers")
        for hand_drive_blockers_item_data in _hand_drive_blockers:
            hand_drive_blockers_item = HandDriveBlocker.from_dict(
                hand_drive_blockers_item_data
            )

            hand_drive_blockers.append(hand_drive_blockers_item)

        def _parse_local_goal_id(data: object) -> None | str:
            if data is None:
                return data
            return cast(None | str, data)

        local_goal_id = _parse_local_goal_id(d.pop("local_goal_id"))

        def _parse_local_goal_id_raw(data: object) -> None | str:
            if data is None:
                return data
            return cast(None | str, data)

        local_goal_id_raw = _parse_local_goal_id_raw(d.pop("local_goal_id_raw"))

        def _parse_mode(
            data: object,
        ) -> (
            WorkModeType0
            | WorkModeType1
            | WorkModeType2
            | WorkModeType3
            | WorkModeType4
            | WorkModeType5
        ):
            try:
                if not isinstance(data, str):
                    raise TypeError()
                componentsschemas_work_mode_type_0 = WorkModeType0(data)

                return componentsschemas_work_mode_type_0
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            try:
                if not isinstance(data, str):
                    raise TypeError()
                componentsschemas_work_mode_type_1 = WorkModeType1(data)

                return componentsschemas_work_mode_type_1
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            try:
                if not isinstance(data, str):
                    raise TypeError()
                componentsschemas_work_mode_type_2 = WorkModeType2(data)

                return componentsschemas_work_mode_type_2
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            try:
                if not isinstance(data, str):
                    raise TypeError()
                componentsschemas_work_mode_type_3 = WorkModeType3(data)

                return componentsschemas_work_mode_type_3
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            try:
                if not isinstance(data, str):
                    raise TypeError()
                componentsschemas_work_mode_type_4 = WorkModeType4(data)

                return componentsschemas_work_mode_type_4
            except (TypeError, ValueError, AttributeError, KeyError):
                pass
            if not isinstance(data, dict):
                raise TypeError()
            componentsschemas_work_mode_type_5 = WorkModeType5.from_dict(data)

            return componentsschemas_work_mode_type_5

        mode = _parse_mode(d.pop("mode"))

        motors = MotorState(d.pop("motors"))

        motors_source = MotorSource(d.pop("motors_source"))

        navigation = NavigationState(d.pop("navigation"))

        remote_control = d.pop("remote_control")

        transitions = []
        _transitions = d.pop("transitions")
        for transitions_item_data in _transitions:
            transitions_item = ChassisTransition.from_dict(transitions_item_data)

            transitions.append(transitions_item)

        def _parse_auto_status_raw(data: object) -> None | str | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(None | str | Unset, data)

        auto_status_raw = _parse_auto_status_raw(d.pop("auto_status_raw", UNSET))

        def _parse_standby(data: object) -> bool | None | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(bool | None | Unset, data)

        standby = _parse_standby(d.pop("standby", UNSET))

        def _parse_standby_raw(data: object) -> None | str | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(None | str | Unset, data)

        standby_raw = _parse_standby_raw(d.pop("standby_raw", UNSET))

        def _parse_stop_status_raw(data: object) -> None | str | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(None | str | Unset, data)

        stop_status_raw = _parse_stop_status_raw(d.pop("stop_status_raw", UNSET))

        def _parse_strip_status_raw(data: object) -> None | str | Unset:
            if data is None:
                return data
            if isinstance(data, Unset):
                return data
            return cast(None | str | Unset, data)

        strip_status_raw = _parse_strip_status_raw(d.pop("strip_status_raw", UNSET))

        chassis_control_state = cls(
            bumper=bumper,
            can_hand_drive=can_hand_drive,
            emergency_switch=emergency_switch,
            hand_drive_blockers=hand_drive_blockers,
            local_goal_id=local_goal_id,
            local_goal_id_raw=local_goal_id_raw,
            mode=mode,
            motors=motors,
            motors_source=motors_source,
            navigation=navigation,
            remote_control=remote_control,
            transitions=transitions,
            auto_status_raw=auto_status_raw,
            standby=standby,
            standby_raw=standby_raw,
            stop_status_raw=stop_status_raw,
            strip_status_raw=strip_status_raw,
        )

        chassis_control_state.additional_properties = d
        return chassis_control_state

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
