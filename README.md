# DroneKit Python

A Python library for communicating with drones via MAVLink protocol. This library provides a high-level API for drone programming, making autonomous flight operations accessible and easy to implement.

## Features

- **Easy Connection**: Connect to drones via Serial, TCP, or UDP
- **High-Level API**: Intuitive Python interface for common drone operations
- **Real-Time Telemetry**: Access GPS, battery, attitude, and other sensor data
- **Autonomous Flight**: Arm, takeoff, navigate, and land programmatically
- **Mode Management**: Switch between flight modes (GUIDED, AUTO, RTL, etc.)
- **Event Callbacks**: Register listeners for attribute changes
- **Robust Error Handling**: Comprehensive exception hierarchy for error management
- **Thread-Safe**: Safe concurrent access to vehicle state
- **Well-Tested**: Comprehensive unit test coverage

## Installation

### From source:

```bash
git clone https://github.com/cywf/dronekit.git
cd dronekit
pip install -e .
```

### Development installation:

```bash
pip install -e ".[dev]"
```

## Requirements

- Python 3.7 or higher
- pymavlink >= 2.4.37
- pyserial >= 3.5
- monotonic >= 1.6

## Quick Start

### Basic Connection and Telemetry

```python
from dronekit import connect
import time

# Connect to vehicle
vehicle = connect('127.0.0.1:14550', wait_ready=True)

# Display telemetry
print(f"Mode: {vehicle.mode.name}")
print(f"Armed: {vehicle.armed}")
print(f"Location: {vehicle.location}")
print(f"GPS: {vehicle.gps_0}")
print(f"Battery: {vehicle.battery}")

# Close connection
vehicle.close()
```

### Simple Takeoff

```python
from dronekit import connect

# Connect
vehicle = connect('127.0.0.1:14550', wait_ready=True)

# Set mode and arm
vehicle.mode = "GUIDED"
vehicle.arm(wait=True)

# Take off to 10 meters
vehicle.simple_takeoff(10)

# Wait for altitude
while vehicle.location.alt < 9.5:
    print(f"Altitude: {vehicle.location.alt:.1f}m")
    time.sleep(1)

# Land
vehicle.mode = "LAND"
vehicle.close()
```

### Navigate to Location

```python
from dronekit import connect, Location

vehicle = connect('127.0.0.1:14550', wait_ready=True)

# Define target location
target = Location(lat=47.6, lon=-122.3, alt=10)

# Navigate to location
vehicle.simple_goto(target, groundspeed=5)

# Monitor progress
while True:
    current = vehicle.location
    distance = get_distance_metres(current, target)
    if distance < 1.0:
        break
    time.sleep(1)

vehicle.close()
```

## Connection Strings

DroneKit supports various connection types:

- **UDP**: `'127.0.0.1:14550'` (default SITL)
- **TCP**: `'tcp:127.0.0.1:5760'`
- **Serial**: `'/dev/ttyUSB0'` or `'COM3'`
- **Serial with baud rate**: `'/dev/ttyUSB0:57600'`

## API Documentation

### Vehicle Class

The main `Vehicle` class provides access to drone functionality:

#### Properties

- `vehicle.armed` - Get/set armed state (bool)
- `vehicle.mode` - Get/set flight mode (VehicleMode)
- `vehicle.location` - Current GPS location (Location)
- `vehicle.gps_0` - GPS information (GPSInfo)
- `vehicle.battery` - Battery information (Battery)
- `vehicle.groundspeed` - Ground speed in m/s (float)
- `vehicle.airspeed` - Air speed in m/s (float)
- `vehicle.heading` - Heading in degrees (int)
- `vehicle.is_armable` - Check if vehicle can be armed (bool)

#### Methods

- `vehicle.connect(wait_ready=False, timeout=30.0)` - Connect to vehicle
- `vehicle.arm(wait=False, timeout=30.0)` - Arm the vehicle
- `vehicle.disarm(wait=False, timeout=30.0)` - Disarm the vehicle
- `vehicle.simple_takeoff(altitude)` - Take off to specified altitude
- `vehicle.simple_goto(location, groundspeed=None)` - Navigate to location
- `vehicle.add_attribute_listener(attr_name, callback)` - Add listener for attribute changes
- `vehicle.remove_attribute_listener(attr_name, callback)` - Remove attribute listener
- `vehicle.close()` - Close connection to vehicle

### Data Classes

#### Location
Represents a geographic location:
```python
location = Location(lat=47.6, lon=-122.3, alt=100.0)
```

#### VehicleMode
Represents flight mode:
```python
mode = VehicleMode("GUIDED")
```

Supported modes:
- STABILIZE
- ACRO
- ALT_HOLD
- AUTO
- GUIDED
- LOITER
- RTL (Return to Launch)
- CIRCLE
- LAND

#### GPSInfo
GPS information container with:
- `fix_type` - GPS fix type (0=No GPS, 2=2D fix, 3=3D fix)
- `num_sat` - Number of satellites
- `eph` - Horizontal dilution of precision
- `epv` - Vertical dilution of precision

#### Battery
Battery information with:
- `voltage` - Battery voltage in volts
- `current` - Current draw in amps
- `level` - Battery level percentage

## Examples

See the `examples/` directory for more detailed examples:

- `basic_telemetry.py` - Connect and monitor telemetry
- `simple_takeoff.py` - Complete takeoff and landing sequence
- `goto_location.py` - Navigate to specific GPS coordinates

## Testing

Run tests with pytest:

```bash
# Install dev dependencies
pip install -e ".[dev]"

# Run all tests
pytest

# Run with coverage
pytest --cov=dronekit --cov-report=html

# Run specific test file
pytest tests/test_vehicle.py
```

## Error Handling

DroneKit provides comprehensive exception handling:

```python
from dronekit import connect
from dronekit.exceptions import (
    ConnectionError,
    TimeoutError,
    VehicleNotConnectedError,
    ArmingError,
    CommandError
)

try:
    vehicle = connect('127.0.0.1:14550', wait_ready=True, timeout=30)
    vehicle.arm(wait=True)
except ConnectionError as e:
    print(f"Failed to connect: {e}")
except ArmingError as e:
    print(f"Failed to arm: {e}")
except TimeoutError as e:
    print(f"Operation timed out: {e}")
finally:
    vehicle.close()
```

## Using with SITL (Software In The Loop)

For testing, you can use ArduPilot SITL:

```bash
# Install SITL
pip install pymavlink MAVProxy

# Run SITL
sim_vehicle.py -v ArduCopter --console --map

# In another terminal, run your DroneKit script
python examples/basic_telemetry.py
```

## Thread Safety

DroneKit uses threading internally for message reception. All public methods and properties are thread-safe and can be safely called from multiple threads.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Related Projects

- [DroneKit-Python](https://github.com/dronekit/dronekit-python) - Original DroneKit library
- [MAVLink](https://mavlink.io/) - MAVLink protocol
- [ArduPilot](https://ardupilot.org/) - Open source autopilot
- [PX4](https://px4.io/) - Professional autopilot

## Support

For issues, questions, or contributions, please use the GitHub issue tracker.

## Acknowledgments

- MAVLink protocol developers
- ArduPilot and PX4 communities
- Original DroneKit-Python contributors
