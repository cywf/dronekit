"""
Core Vehicle class for DroneKit library.
"""

import time
import threading
from typing import Optional, Callable, Dict
from pymavlink import mavutil
from .exceptions import (
    ConnectionError,
    TimeoutError,
    VehicleNotConnectedError,
    ArmingError,
    CommandError,
)
from .logger import setup_logger


logger = setup_logger(__name__)


class Location:
    """Represents a geographic location."""

    def __init__(self, lat: float = 0.0, lon: float = 0.0, alt: float = 0.0):
        """
        Initialize a location.

        Args:
            lat: Latitude in degrees
            lon: Longitude in degrees
            alt: Altitude in meters
        """
        self.lat = lat
        self.lon = lon
        self.alt = alt

    def __repr__(self):
        return f"Location(lat={self.lat}, lon={self.lon}, alt={self.alt})"


class GPSInfo:
    """GPS information container."""

    def __init__(self):
        self.fix_type = 0
        self.num_sat = 0
        self.eph = 0
        self.epv = 0

    def __repr__(self):
        return f"GPSInfo(fix={self.fix_type}, sats={self.num_sat})"


class Battery:
    """Battery information container."""

    def __init__(self):
        self.voltage = 0.0
        self.current = 0.0
        self.level = 0

    def __repr__(self):
        return f"Battery(voltage={self.voltage}V, level={self.level}%)"


class VehicleMode:
    """Vehicle flight mode."""

    def __init__(self, name: str = "UNKNOWN"):
        self.name = name

    def __repr__(self):
        return f"VehicleMode({self.name})"

    def __eq__(self, other):
        if isinstance(other, str):
            return self.name == other
        elif isinstance(other, VehicleMode):
            return self.name == other.name
        return False


class Vehicle:
    """
    Main vehicle class for interacting with a drone via MAVLink.

    This class provides methods to connect to a vehicle, access telemetry data,
    and send commands for autonomous flight operations.
    """

    def __init__(self, connection_string: str):
        """
        Initialize a vehicle instance.

        Args:
            connection_string: MAVLink connection string (e.g., '127.0.0.1:14550', '/dev/ttyUSB0')
        """
        self._connection_string = connection_string
        self._master: Optional[mavutil.mavlink_connection] = None
        self._connected = False
        self._armed = False
        self._mode = VehicleMode("UNKNOWN")
        self._location = Location()
        self._gps = GPSInfo()
        self._battery = Battery()
        self._groundspeed = 0.0
        self._airspeed = 0.0
        self._heading = 0
        self._is_armable = False
        self._system_status = "UNKNOWN"

        # Threading for message reception
        self._receive_thread: Optional[threading.Thread] = None
        self._running = False
        self._lock = threading.Lock()

        # Callbacks
        self._attribute_listeners: Dict[str, list] = {}

        logger.info(f"Vehicle instance created for {connection_string}")

    def connect(self, wait_ready: bool = False, timeout: float = 30.0):
        """
        Connect to the vehicle.

        Args:
            wait_ready: Wait for vehicle to be ready before returning
            timeout: Connection timeout in seconds

        Raises:
            ConnectionError: If connection fails
            TimeoutError: If connection times out
        """
        try:
            logger.info(f"Connecting to vehicle at {self._connection_string}")
            self._master = mavutil.mavlink_connection(self._connection_string)

            # Wait for heartbeat
            logger.info("Waiting for heartbeat...")
            start_time = time.time()
            while time.time() - start_time < timeout:
                msg = self._master.recv_match(
                    type="HEARTBEAT", blocking=True, timeout=1.0
                )
                if msg:
                    logger.info(f"Heartbeat received from system {msg.get_srcSystem()}")
                    self._connected = True
                    break

            if not self._connected:
                raise TimeoutError(f"No heartbeat received within {timeout} seconds")

            # Start message receiving thread
            self._running = True
            self._receive_thread = threading.Thread(
                target=self._receive_loop, daemon=True
            )
            self._receive_thread.start()

            # Wait for essential data if requested
            if wait_ready:
                self._wait_ready(timeout)

            logger.info("Successfully connected to vehicle")

        except Exception as e:
            logger.error(f"Connection failed: {e}")
            raise ConnectionError(f"Failed to connect to vehicle: {e}")

    def _wait_ready(self, timeout: float = 30.0):
        """Wait for vehicle to be ready with essential data."""
        logger.info("Waiting for vehicle to be ready...")
        start_time = time.time()

        while time.time() - start_time < timeout:
            # Check if we have GPS and basic telemetry
            if self._gps.fix_type >= 2 and self._system_status != "UNKNOWN":
                logger.info("Vehicle is ready")
                return
            time.sleep(0.5)

        logger.warning("Timeout waiting for vehicle ready state")

    def _receive_loop(self):
        """Background thread to receive and process MAVLink messages."""
        logger.debug("Message receive loop started")

        while self._running and self._master:
            try:
                msg = self._master.recv_match(blocking=True, timeout=1.0)
                if msg:
                    self._process_message(msg)
            except Exception as e:
                logger.error(f"Error in receive loop: {e}")
                if not self._running:
                    break

        logger.debug("Message receive loop ended")

    def _process_message(self, msg):
        """Process incoming MAVLink messages."""
        msg_type = msg.get_type()

        try:
            if msg_type == "HEARTBEAT":
                self._process_heartbeat(msg)
            elif msg_type == "GLOBAL_POSITION_INT":
                self._process_global_position(msg)
            elif msg_type == "GPS_RAW_INT":
                self._process_gps_raw(msg)
            elif msg_type == "SYS_STATUS":
                self._process_sys_status(msg)
            elif msg_type == "ATTITUDE":
                self._process_attitude(msg)
            elif msg_type == "VFR_HUD":
                self._process_vfr_hud(msg)
        except Exception as e:
            logger.error(f"Error processing {msg_type} message: {e}")

    def _process_heartbeat(self, msg):
        """Process HEARTBEAT message."""
        with self._lock:
            # Update armed state
            old_armed = self._armed
            self._armed = (
                msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            ) != 0

            if old_armed != self._armed:
                logger.info(f"Vehicle armed state changed: {self._armed}")
                self._notify_attribute_listeners("armed")

            # Update mode
            mode_mapping = {
                0: "STABILIZE",
                1: "ACRO",
                2: "ALT_HOLD",
                3: "AUTO",
                4: "GUIDED",
                5: "LOITER",
                6: "RTL",
                7: "CIRCLE",
                9: "LAND",
            }

            old_mode = self._mode.name
            custom_mode = msg.custom_mode
            self._mode = VehicleMode(
                mode_mapping.get(custom_mode, f"MODE_{custom_mode}")
            )

            if old_mode != self._mode.name:
                logger.info(f"Vehicle mode changed: {self._mode.name}")
                self._notify_attribute_listeners("mode")

            # Update system status
            status_mapping = {
                0: "UNINIT",
                1: "BOOT",
                2: "CALIBRATING",
                3: "STANDBY",
                4: "ACTIVE",
                5: "CRITICAL",
                6: "EMERGENCY",
                7: "POWEROFF",
            }
            self._system_status = status_mapping.get(msg.system_status, "UNKNOWN")
            self._is_armable = self._system_status in ["STANDBY", "ACTIVE"]

    def _process_global_position(self, msg):
        """Process GLOBAL_POSITION_INT message."""
        with self._lock:
            self._location = Location(
                lat=msg.lat / 1e7, lon=msg.lon / 1e7, alt=msg.alt / 1000.0
            )
            self._notify_attribute_listeners("location")

    def _process_gps_raw(self, msg):
        """Process GPS_RAW_INT message."""
        with self._lock:
            self._gps.fix_type = msg.fix_type
            self._gps.num_sat = msg.satellites_visible
            self._gps.eph = msg.eph / 100.0 if msg.eph != 65535 else 0
            self._gps.epv = msg.epv / 100.0 if msg.epv != 65535 else 0
            self._notify_attribute_listeners("gps_0")

    def _process_sys_status(self, msg):
        """Process SYS_STATUS message."""
        with self._lock:
            self._battery.voltage = msg.voltage_battery / 1000.0
            self._battery.current = (
                msg.current_battery / 100.0 if msg.current_battery != -1 else 0
            )
            self._battery.level = (
                msg.battery_remaining if msg.battery_remaining != -1 else 0
            )
            self._notify_attribute_listeners("battery")

    def _process_attitude(self, msg):
        """Process ATTITUDE message."""
        with self._lock:
            # Convert yaw to heading (0-360 degrees)
            import math

            heading = math.degrees(msg.yaw)
            if heading < 0:
                heading += 360
            self._heading = int(heading)
            self._notify_attribute_listeners("heading")

    def _process_vfr_hud(self, msg):
        """Process VFR_HUD message."""
        with self._lock:
            self._groundspeed = msg.groundspeed
            self._airspeed = msg.airspeed
            self._notify_attribute_listeners("groundspeed")
            self._notify_attribute_listeners("airspeed")

    def _notify_attribute_listeners(self, attr_name: str):
        """Notify listeners of attribute changes."""
        if attr_name in self._attribute_listeners:
            for callback in self._attribute_listeners[attr_name]:
                try:
                    callback(self, attr_name, getattr(self, attr_name))
                except Exception as e:
                    logger.error(f"Error in attribute listener callback: {e}")

    def add_attribute_listener(self, attr_name: str, callback: Callable):
        """
        Add a listener for attribute changes.

        Args:
            attr_name: Name of the attribute to listen to
            callback: Function to call when attribute changes
        """
        if attr_name not in self._attribute_listeners:
            self._attribute_listeners[attr_name] = []
        self._attribute_listeners[attr_name].append(callback)

    def remove_attribute_listener(self, attr_name: str, callback: Callable):
        """
        Remove an attribute listener.

        Args:
            attr_name: Name of the attribute
            callback: Callback function to remove
        """
        if attr_name in self._attribute_listeners:
            try:
                self._attribute_listeners[attr_name].remove(callback)
            except ValueError:
                pass

    @property
    def armed(self) -> bool:
        """Get armed state of the vehicle."""
        self._check_connected()
        with self._lock:
            return self._armed

    @armed.setter
    def armed(self, value: bool):
        """
        Set armed state of the vehicle.

        Args:
            value: True to arm, False to disarm

        Raises:
            ArmingError: If arming/disarming fails
        """
        self._check_connected()

        if value:
            self.arm(wait=True)
        else:
            self.disarm(wait=True)

    def arm(self, wait: bool = False, timeout: float = 30.0):
        """
        Arm the vehicle.

        Args:
            wait: Wait for arming to complete
            timeout: Timeout in seconds

        Raises:
            ArmingError: If arming fails
        """
        self._check_connected()

        if not self._is_armable:
            raise ArmingError("Vehicle is not armable. Check GPS fix and calibration.")

        logger.info("Arming vehicle...")

        # Send arm command
        self._master.mav.command_long_send(
            self._master.target_system,
            self._master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            1,  # arm
            0,
            0,
            0,
            0,
            0,
            0,
        )

        if wait:
            start_time = time.time()
            while time.time() - start_time < timeout:
                if self._armed:
                    logger.info("Vehicle armed successfully")
                    return
                time.sleep(0.5)
            raise ArmingError(f"Vehicle did not arm within {timeout} seconds")

    def disarm(self, wait: bool = False, timeout: float = 30.0):
        """
        Disarm the vehicle.

        Args:
            wait: Wait for disarming to complete
            timeout: Timeout in seconds
        """
        self._check_connected()
        logger.info("Disarming vehicle...")

        # Send disarm command
        self._master.mav.command_long_send(
            self._master.target_system,
            self._master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            0,  # disarm
            0,
            0,
            0,
            0,
            0,
            0,
        )

        if wait:
            start_time = time.time()
            while time.time() - start_time < timeout:
                if not self._armed:
                    logger.info("Vehicle disarmed successfully")
                    return
                time.sleep(0.5)

    def simple_takeoff(self, altitude: float):
        """
        Take off to a specified altitude.

        Args:
            altitude: Target altitude in meters

        Raises:
            CommandError: If takeoff command fails
        """
        self._check_connected()

        if not self._armed:
            raise CommandError("Vehicle must be armed before takeoff")

        logger.info(f"Taking off to {altitude}m...")

        # Send takeoff command
        self._master.mav.command_long_send(
            self._master.target_system,
            self._master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,  # confirmation
            0,
            0,
            0,
            0,
            0,
            0,
            altitude,
        )

        logger.info("Takeoff command sent")

    def simple_goto(self, location: Location, groundspeed: Optional[float] = None):
        """
        Go to a specified location.

        Args:
            location: Target location
            groundspeed: Optional groundspeed in m/s
        """
        self._check_connected()

        logger.info(f"Going to {location}")

        if groundspeed:
            # Set groundspeed
            self._master.mav.command_long_send(
                self._master.target_system,
                self._master.target_component,
                mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                0,  # confirmation
                1,  # speed type (groundspeed)
                groundspeed,
                -1,
                0,
                0,
                0,
                0,
            )

        # Send position target
        self._master.mav.set_position_target_global_int_send(
            0,  # timestamp
            self._master.target_system,
            self._master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,  # type_mask (only positions enabled)
            int(location.lat * 1e7),
            int(location.lon * 1e7),
            location.alt,
            0,
            0,
            0,  # velocity
            0,
            0,
            0,  # acceleration
            0,
            0,  # yaw, yaw_rate
        )

    @property
    def mode(self) -> VehicleMode:
        """Get current vehicle mode."""
        self._check_connected()
        with self._lock:
            return self._mode

    @mode.setter
    def mode(self, mode: str):
        """
        Set vehicle mode.

        Args:
            mode: Mode name (e.g., 'GUIDED', 'AUTO', 'RTL')
        """
        self._check_connected()

        mode_mapping = {
            "STABILIZE": 0,
            "ACRO": 1,
            "ALT_HOLD": 2,
            "AUTO": 3,
            "GUIDED": 4,
            "LOITER": 5,
            "RTL": 6,
            "CIRCLE": 7,
            "LAND": 9,
        }

        if mode not in mode_mapping:
            raise CommandError(f"Unknown mode: {mode}")

        logger.info(f"Setting mode to {mode}")

        self._master.mav.set_mode_send(
            self._master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_mapping[mode],
        )

    @property
    def location(self) -> Location:
        """Get current vehicle location."""
        self._check_connected()
        with self._lock:
            return self._location

    @property
    def gps_0(self) -> GPSInfo:
        """Get GPS information."""
        self._check_connected()
        with self._lock:
            return self._gps

    @property
    def battery(self) -> Battery:
        """Get battery information."""
        self._check_connected()
        with self._lock:
            return self._battery

    @property
    def groundspeed(self) -> float:
        """Get groundspeed in m/s."""
        self._check_connected()
        with self._lock:
            return self._groundspeed

    @property
    def airspeed(self) -> float:
        """Get airspeed in m/s."""
        self._check_connected()
        with self._lock:
            return self._airspeed

    @property
    def heading(self) -> int:
        """Get heading in degrees."""
        self._check_connected()
        with self._lock:
            return self._heading

    @property
    def is_armable(self) -> bool:
        """Check if vehicle is armable."""
        self._check_connected()
        with self._lock:
            return self._is_armable

    def _check_connected(self):
        """Check if vehicle is connected."""
        if not self._connected:
            raise VehicleNotConnectedError("Vehicle is not connected")

    def close(self):
        """Close the connection to the vehicle."""
        if self._running:
            logger.info("Closing vehicle connection...")
            self._running = False

            if self._receive_thread:
                self._receive_thread.join(timeout=2.0)

            if self._master:
                self._master.close()
                self._master = None

            self._connected = False
            logger.info("Vehicle connection closed")

    def __enter__(self):
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close()

    def __repr__(self):
        status = "connected" if self._connected else "disconnected"
        return f"Vehicle({self._connection_string}, {status})"


def connect(
    connection_string: str, wait_ready: bool = False, timeout: float = 30.0
) -> Vehicle:
    """
    Connect to a vehicle.

    Args:
        connection_string: MAVLink connection string
        wait_ready: Wait for vehicle to be ready
        timeout: Connection timeout in seconds

    Returns:
        Connected Vehicle instance

    Example:
        >>> vehicle = connect('127.0.0.1:14550', wait_ready=True)
        >>> print(vehicle.location)
        >>> vehicle.close()
    """
    vehicle = Vehicle(connection_string)
    vehicle.connect(wait_ready=wait_ready, timeout=timeout)
    return vehicle
