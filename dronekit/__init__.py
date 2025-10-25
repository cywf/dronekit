"""
DroneKit Python - Python library for communicating with drones via MAVLink.

This library provides a high-level API for drone programming, making it easy to:
- Connect to drones using various connection types (serial, UDP, TCP)
- Access telemetry data and vehicle state
- Send commands for autonomous flight operations
- Manage missions and waypoints
- Handle events and callbacks

Example:
    >>> from dronekit import connect
    >>> vehicle = connect('127.0.0.1:14550', wait_ready=True)
    >>> print(f"GPS: {vehicle.gps_0}")
    >>> vehicle.armed = True
    >>> vehicle.simple_takeoff(10)
    >>> vehicle.close()
"""

__version__ = '0.1.0'
__author__ = 'Ky1o P'
__license__ = 'MIT'

from .vehicle import Vehicle, connect
from .exceptions import (
    DroneKitException,
    ConnectionError,
    TimeoutError,
    CommandError,
    VehicleNotConnectedError,
)

__all__ = [
    'Vehicle',
    'connect',
    'DroneKitException',
    'ConnectionError',
    'TimeoutError',
    'CommandError',
    'VehicleNotConnectedError',
    '__version__',
]
