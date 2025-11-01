"""
Example: Simple takeoff and landing.

This example demonstrates how to:
- Connect to a vehicle
- Arm the vehicle
- Take off to a specified altitude
- Wait at altitude
- Land the vehicle
"""

from dronekit import connect
import time


def main():
    """Main example function."""
    connection_string = "127.0.0.1:14550"  # SITL default
    target_altitude = 10  # meters

    print(f"Connecting to vehicle on {connection_string}...")

    try:
        # Connect to vehicle
        vehicle = connect(connection_string, wait_ready=True, timeout=30)
        print("Successfully connected!")

        # Check if vehicle is armable
        print("\nChecking if vehicle is armable...")
        if not vehicle.is_armable:
            print("Vehicle is not armable. Check GPS fix and calibration.")
            vehicle.close()
            return 1

        # Set mode to GUIDED
        print("Setting mode to GUIDED...")
        vehicle.mode = "GUIDED"
        time.sleep(2)

        # Arm the vehicle
        print("Arming vehicle...")
        vehicle.arm(wait=True, timeout=30)
        print("Vehicle armed!")

        # Take off
        print(f"Taking off to {target_altitude}m...")
        vehicle.simple_takeoff(target_altitude)

        # Wait until target altitude is reached
        while True:
            current_altitude = vehicle.location.alt
            print(f"Altitude: {current_altitude:.1f}m / {target_altitude}m")

            if current_altitude >= target_altitude * 0.95:
                print("Target altitude reached!")
                break

            time.sleep(1)

        # Hold position for 10 seconds
        print("\nHolding position for 10 seconds...")
        time.sleep(10)

        # Land
        print("Landing...")
        vehicle.mode = "LAND"

        # Wait until landed
        while vehicle.armed:
            current_altitude = vehicle.location.alt
            print(f"Altitude: {current_altitude:.1f}m")
            time.sleep(1)

        print("Landed and disarmed!")

        # Close connection
        print("\nClosing connection...")
        vehicle.close()
        print("Connection closed.")

    except Exception as e:
        print(f"Error: {e}")
        return 1

    return 0


if __name__ == "__main__":
    exit(main())
