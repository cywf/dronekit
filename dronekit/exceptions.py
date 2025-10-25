"""
Custom exceptions for DroneKit library.
"""


class DroneKitException(Exception):
    """Base exception for all DroneKit errors."""
    pass


class ConnectionError(DroneKitException):
    """Raised when connection to vehicle fails."""
    pass


class TimeoutError(DroneKitException):
    """Raised when an operation times out."""
    pass


class CommandError(DroneKitException):
    """Raised when a command fails to execute."""
    pass


class VehicleNotConnectedError(DroneKitException):
    """Raised when trying to perform operations on a disconnected vehicle."""
    pass


class ArmingError(CommandError):
    """Raised when arming the vehicle fails."""
    pass


class MissionError(DroneKitException):
    """Raised when mission-related operations fail."""
    pass


class ParameterError(DroneKitException):
    """Raised when parameter operations fail."""
    pass
