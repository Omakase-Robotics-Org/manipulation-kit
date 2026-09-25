from enum import Enum


class ChassisCatalogRequestType5Kind(str, Enum):
    ROS_STATUS = "ros_status"

    def __str__(self) -> str:
        return str(self.value)
