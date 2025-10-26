"""
Example: Goto location.

This example demonstrates how to:
- Connect to a vehicle
- Navigate to a specific GPS coordinate
- Monitor progress
"""

from dronekit import connect, Location
import time
import math


def get_distance_metres(location1, location2):
    """
    Calculate distance between two locations in meters.

    This method is an approximation and will not be accurate over large distances.
    """
    dlat = location2.lat - location1.lat
    dlon = location2.lon - location1.lon
    return math.sqrt((dlat * dlat) + (dlon * dlon)) * 1.113195e5


def main():
    """Main example function."""
    connection_string = "127.0.0.1:14550"  # SITL default

    print(f"Connecting to vehicle on {connection_string}...")

    try:
        # Connect to vehicle
        vehicle = connect(connection_string, wait_ready=True, timeout=30)
        print("Successfully connected!")

        # Get current location
        current_location = vehicle.location
        print(f"\nCurrent location: {current_location}")

        # Set mode to GUIDED
        print("\nSetting mode to GUIDED...")
        vehicle.mode = "GUIDED"
        time.sleep(2)

        # Arm the vehicle if not already armed
        if not vehicle.armed:
            print("Arming vehicle...")
            vehicle.arm(wait=True, timeout=30)
            print("Vehicle armed!")

            # Take off to 10m
            print("Taking off to 10m...")
            vehicle.simple_takeoff(10)

            while vehicle.location.alt < 9.5:
                print(f"Altitude: {vehicle.location.alt:.1f}m")
                time.sleep(1)

        # Define target location (10m north of current location)
        # Note: 0.0001 degrees latitude ≈ 11.1 meters
        target_location = Location(
            lat=current_location.lat + 0.0001, lon=current_location.lon, alt=10
        )

        print(f"\nGoing to target location: {target_location}")
        vehicle.simple_goto(target_location, groundspeed=5)

        # Monitor progress
        while True:
            current = vehicle.location
            distance = get_distance_metres(current, target_location)

            print(
                f"Distance to target: {distance:.1f}m | "
                f"Alt: {current.alt:.1f}m | "
                f"Speed: {vehicle.groundspeed:.1f}m/s"
            )

            if distance < 1.0:
                print("Target reached!")
                break

            time.sleep(1)

        # Hold for 5 seconds
        print("\nHolding position for 5 seconds...")
        time.sleep(5)

        # Return to launch
        print("\nReturning to launch...")
        vehicle.mode = "RTL"

        # Wait a bit
        time.sleep(10)

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
