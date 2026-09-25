from enum import Enum


class ErrorKind(str, Enum):
    AUTO_CHARGING = "auto_charging"
    DEVICE = "device"
    DOCK_UNCONFIGURED = "dock_unconfigured"
    HAND_DRIVE_BLOCKED = "hand_drive_blocked"
    INVALID = "invalid"
    KILL_LATCHED = "kill_latched"
    LEASE_HELD = "lease_held"
    MANUAL_CHARGING = "manual_charging"
    MANUAL_MODE_REQUIRED = "manual_mode_required"
    MAPPING_MODE = "mapping_mode"
    MODE_NOT_TAKEN = "mode_not_taken"
    NAVIGATION_DISABLED = "navigation_disabled"
    NAVIGATION_ENABLED = "navigation_enabled"
    NOT_FOUND = "not_found"
    PRECONDITION_UNMET = "precondition_unmet"
    REFUSED = "refused"
    REMOTE_CONTROL_FAILED = "remote_control_failed"
    REMOTE_CONTROL_MODE = "remote_control_mode"
    ROS_UNMAPPED = "ros_unmapped"
    SETTINGS_CONFLICT = "settings_conflict"
    SETTINGS_READBACK_FAILED = "settings_readback_failed"
    TIMEOUT = "timeout"
    UNAUTHENTICATED = "unauthenticated"
    UNAVAILABLE = "unavailable"
    UPDATE_IN_PROGRESS = "update_in_progress"
    VENDOR = "vendor"

    def __str__(self) -> str:
        return str(self.value)
