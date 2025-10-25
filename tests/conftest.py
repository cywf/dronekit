"""Configuration file for pytest."""

import pytest


@pytest.fixture
def sample_location():
    """Fixture providing a sample location."""
    from dronekit.vehicle import Location
    return Location(lat=47.6, lon=-122.3, alt=100.0)


@pytest.fixture
def sample_vehicle():
    """Fixture providing a sample vehicle instance."""
    from dronekit.vehicle import Vehicle
    vehicle = Vehicle("127.0.0.1:14550")
    vehicle._connected = True  # Simulate connection
    yield vehicle
    vehicle._connected = False
