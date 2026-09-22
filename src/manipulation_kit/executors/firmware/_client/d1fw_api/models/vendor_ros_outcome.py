from enum import Enum


class VendorRosOutcome(str, Enum):
    ACKNOWLEDGED = "acknowledged"
    OUTCOME_UNKNOWN = "outcome_unknown"
    REFUSED = "refused"
    TRANSPORT_SENT = "transport_sent"

    def __str__(self) -> str:
        return str(self.value)
