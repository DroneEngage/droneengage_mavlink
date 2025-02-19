################################################################################
# 
# Author: M. Hefny  (mohammad.hefny@gmail.com)  
# Date: Oct 2024
# 
# Description:
# This script sends predefined distance sensor data over a MAVLink UDP connection.
#
# Usage:
# python send_distance.py <udp_ip> <udp_port> [rate]
#
# Parameters:
# <udp_ip>        : Destination IP address for the MAVLink connection (e.g., 127.0.0.1)
# <udp_port>      : Destination port number for the MAVLink connection (e.g., 14551)
# [rate]          : (Optional) Delay between sending data in seconds (default: 1.0)
#
# Example:
# python send_distance.py 127.0.0.1 14551 1.5
#
################################################################################

import sys
import time
from pymavlink import mavutil

# Predefined array of distances for the sensors (current_distance, sensor_id, orientation)
PREDEFINED_DISTANCES = [(1, 0, 0), (10, 2, 2),
                        (3, 4, 4), (4, 6, 6)]

def send_distance_sensor_data(ts, the_connection):
    """
    Sends predefined distance sensor data over the MAVLink connection.

    Parameters:
    ts (float): The current timestamp.
    the_connection: The MAVLink connection object.
    """
    for current_distance, sensor_id, orientation in PREDEFINED_DISTANCES:
        time_boot_ms = int(ts * 1000) % 4294967296  # Ensure the value is within uint32 range
        
        # Create the message using the encode method
        msg = the_connection.mav.distance_sensor_encode(
            time_boot_ms=time_boot_ms,
            min_distance=1,  # Minimum distance
            max_distance=20,  # Maximum distance
            current_distance=current_distance,
            type=0,  # Sensor type (0 for generic)
            id=sensor_id,
            orientation=orientation,  # Sensor orientation
            covariance=255  # Measurement variance
        )

        # Send the message over the MAVLink connection
        the_connection.mav.send(msg)

        print(f"Sent message for sensor ID {sensor_id} with current distance {current_distance}")

def main(udp_ip, udp_port, rate):
    """
    Main function to establish the MAVLink connection and send sensor data.

    Parameters:
    udp_ip (str): Destination IP address for the MAVLink connection.
    udp_port (int): Destination port number for the MAVLink connection.
    rate (float): Delay between sending data in seconds.
    """
    # Create a MAVLink connection to send messages
    the_connection = mavutil.mavlink_connection(f'udpout:{udp_ip}:{udp_port}',
                                                source_system=1,      # Set SYSID to 1
                                                source_component=1)   # Set COMPID to 1

    # Main loop to send distance sensor data
    while True:
        ts = time.time()  # Get the current time
        send_distance_sensor_data(ts, the_connection)  # Send distance sensor data
        time.sleep(rate)  # Delay for the specified rate

if __name__ == "__main__":
    # Check for the correct number of command-line arguments
    if len(sys.argv) < 3 or len(sys.argv) > 4:
        print("Usage: python script.py <udp_ip> <udp_port> [rate]")
        sys.exit(1)

    # Parse command line arguments
    udp_ip = sys.argv[1]
    udp_port = int(sys.argv[2])
    
    # Set default rate to 1.0 if not provided
    rate = float(sys.argv[3]) if len(sys.argv) == 4 else 1.0

    # Run the main function
    main(udp_ip, udp_port, rate)
