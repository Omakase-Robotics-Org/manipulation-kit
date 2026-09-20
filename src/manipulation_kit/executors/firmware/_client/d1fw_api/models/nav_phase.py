from enum import Enum


class NavPhase(str, Enum):
    ARRIVED = "arrived"
    DEPARTING = "departing"
    ENSURING_NAV_READY = "ensuring_nav_ready"
    EN_ROUTE = "en_route"
    FAILED = "failed"
    INTERRUPTED = "interrupted"
    TIMEOUT = "timeout"

    def __str__(self) -> str:
        return str(self.value)
