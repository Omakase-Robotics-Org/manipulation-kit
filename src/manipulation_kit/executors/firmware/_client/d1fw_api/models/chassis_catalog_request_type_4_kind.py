from enum import Enum


class ChassisCatalogRequestType4Kind(str, Enum):
    USB = "usb"

    def __str__(self) -> str:
        return str(self.value)
