from enum import Enum


class MappingMapSource(str, Enum):
    MAP = "map"
    MAP_MODIFY = "map_modify"
    UNKNOWN = "unknown"

    def __str__(self) -> str:
        return str(self.value)
