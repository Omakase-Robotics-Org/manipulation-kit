from enum import Enum


class AdvisoryCode(str, Enum):
    ARM_IDLE_RECOVER_SUGGESTED = "arm_idle_recover_suggested"
    SLIDER_COMMISSIONING_REQUIRED = "slider_commissioning_required"
    SLIDER_ZERO_VERIFICATION_PENDING = "slider_zero_verification_pending"

    def __str__(self) -> str:
        return str(self.value)
