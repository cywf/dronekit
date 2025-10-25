"""
Example: Basic vehicle connection and telemetry monitoring.

This example demonstrates how to:
- Connect to a vehicle
- Monitor basic telemetry data
- Close the connection properly
"""

from dronekit import connect
import time


def main():
    """Main example function."""
    # Connection string examples:
    # - TCP: 'tcp:127.0.0.1:5760'
    # - UDP: '127.0.0.1:14550'
    # - Serial: '/dev/ttyUSB0' or 'COM3' (with baud rate: '/dev/ttyUSB0:57600')
    
    connection_string = '127.0.0.1:14550'  # SITL default
    
    print(f"Connecting to vehicle on {connection_string}...")
    
    try:
        # Connect to vehicle
        vehicle = connect(connection_string, wait_ready=True, timeout=30)
        
        print("Successfully connected!")
        print(f"Vehicle: {vehicle}")
        
        # Display basic telemetry
        print("\n=== Vehicle Telemetry ===")
        print(f"Mode: {vehicle.mode.name}")
        print(f"Armed: {vehicle.armed}")
        print(f"Is Armable: {vehicle.is_armable}")
        print(f"Location: {vehicle.location}")
        print(f"GPS: {vehicle.gps_0}")
        print(f"Battery: {vehicle.battery}")
        print(f"Groundspeed: {vehicle.groundspeed} m/s")
        print(f"Heading: {vehicle.heading} degrees")
        
        # Monitor telemetry for 10 seconds
        print("\nMonitoring telemetry for 10 seconds...")
        start_time = time.time()
        
        while time.time() - start_time < 10:
            print(f"\rAlt: {vehicle.location.alt:.1f}m | "
                  f"Speed: {vehicle.groundspeed:.1f}m/s | "
                  f"Battery: {vehicle.battery.level}%", end='')
            time.sleep(0.5)
        
        print("\n\nClosing connection...")
        vehicle.close()
        print("Connection closed.")
        
    except Exception as e:
        print(f"Error: {e}")
        return 1
    
    return 0


if __name__ == '__main__':
    exit(main())
