"""Unit tests for DroneKit exceptions."""

import pytest
from dronekit.exceptions import (
    DroneKitException,
    ConnectionError,
    TimeoutError,
    CommandError,
    VehicleNotConnectedError,
    ArmingError,
    MissionError,
    ParameterError,
)


def test_base_exception():
    """Test base DroneKit exception."""
    with pytest.raises(DroneKitException):
        raise DroneKitException("Test error")


def test_connection_error():
    """Test ConnectionError exception."""
    with pytest.raises(ConnectionError):
        raise ConnectionError("Connection failed")
    
    # Should also be a DroneKitException
    with pytest.raises(DroneKitException):
        raise ConnectionError("Connection failed")


def test_timeout_error():
    """Test TimeoutError exception."""
    with pytest.raises(TimeoutError):
        raise TimeoutError("Operation timed out")


def test_command_error():
    """Test CommandError exception."""
    with pytest.raises(CommandError):
        raise CommandError("Command failed")


def test_vehicle_not_connected_error():
    """Test VehicleNotConnectedError exception."""
    with pytest.raises(VehicleNotConnectedError):
        raise VehicleNotConnectedError("Vehicle not connected")


def test_arming_error():
    """Test ArmingError exception."""
    with pytest.raises(ArmingError):
        raise ArmingError("Arming failed")
    
    # Should also be a CommandError
    with pytest.raises(CommandError):
        raise ArmingError("Arming failed")


def test_mission_error():
    """Test MissionError exception."""
    with pytest.raises(MissionError):
        raise MissionError("Mission failed")


def test_parameter_error():
    """Test ParameterError exception."""
    with pytest.raises(ParameterError):
        raise ParameterError("Parameter error")
