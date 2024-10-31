################################################################################
# 
# Author: M. Hefny  (mohammad.hefny@gmail.com)  
# Date: Oct 2024
# 
# Description:
# This script sends simulated distance sensor data over a MAVLink UDP connection.
# The distances are generated randomly within specified bounds, or they can be fixed
# if a constant value is provided.
#
# Usage:
# python send_distance.py <udp_ip> <udp_port> <min_distance> <max_distance> [rate] [constant]
#
# Parameters:
# <udp_ip>        : Destination IP address for the MAVLink connection (e.g., 127.0.0.1)
# <udp_port>      : Destination port number for the MAVLink connection (e.g., 14551)
# <min_distance>  : Minimum distance value for the sensors (e.g., 5)
# <max_distance>  : Maximum distance value for the sensors (e.g., 800)
# [rate]          : (Optional) Delay between sending data in seconds (default: 1.0)
# [constant]      : (Optional) Fixed distance value for all sensors (e.g., 100)
#
# Example:
# python send_distance.py 127.0.0.1 14551 5 800 1.5 100
#
################################################################################

import sys
import time
import random
from pymavlink import mavutil

# Global variable to hold distances
distances = []

def generate_distances(min_distance, max_distance):
    """Generate random distances for sensors directly within specified bounds."""
    return [(random.randint(min_distance, max_distance), i, random.randint(0, 7)) for i in range(8)]

def send_distance_sensor_data(ts, the_connection, min_distance, max_distance, constant=None):
    """
    Sends randomly generated distance sensor data or fixed distance data over the MAVLink connection.

    Parameters:
    ts (float): The current timestamp.
    the_connection: The MAVLink connection object.
    min_distance (int): Minimum sensor distance.
    max_distance (int): Maximum sensor distance.
    constant (int, optional): Fixed distance for all sensors. If None, random distances are generated.
    """
    global distances  # Declare distances as global to modify it
    
    # If a constant value is provided, use it for all sensors
    if constant is None:
        distances = generate_distances(min_distance, max_distance)
    
    for current_distance, sensor_id, orientation in distances:
        time_boot_ms = int(ts * 1000) % 4294967296  # Ensure the value is within uint32 range
        
        # Create the message using the encode method
        msg = the_connection.mav.distance_sensor_encode(
            time_boot_ms=time_boot_ms,
            min_distance=min_distance,
            max_distance=max_distance,
            current_distance=current_distance,
            type=0,  # Sensor type (0 for generic)
            id=sensor_id,
            orientation=orientation,  # Sensor orientation
            covariance=255  # Measurement variance
        )

        # Send the message over the MAVLink connection
        the_connection.mav.send(msg)

        print(f"Sent message for sensor ID {sensor_id} with current distance {current_distance}")

def main(udp_ip, udp_port, min_distance, max_distance, rate, constant):
    """
    Main function to establish the MAVLink connection and send sensor data.

    Parameters:
    udp_ip (str): Destination IP address for the MAVLink connection.
    udp_port (int): Destination port number for the MAVLink connection.
    min_distance (int): Minimum sensor distance.
    max_distance (int): Maximum sensor distance.
    rate (float): Delay between sending data in seconds.
    constant (int, optional): Fixed distance for all sensors.
    """
    # Create a MAVLink connection to send messages
    the_connection = mavutil.mavlink_connection(f'udpout:{udp_ip}:{udp_port}',
                                                source_system=1,      # Set SYSID to 1
                                                source_component=1)   # Set COMPID to 1

    # initial values
    global distances  # Declare distances as global to modify it
    distances = generate_distances(min_distance, max_distance)

    # Main loop to send distance sensor data
    while True:
        ts = time.time()  # Get the current time
        send_distance_sensor_data(ts, the_connection, min_distance, max_distance, constant)  # Send distance sensor data
        time.sleep(rate)  # Delay for the specified rate

if __name__ == "__main__":
    # Check for the correct number of command-line arguments
    if len(sys.argv) < 5 or len(sys.argv) > 7:
        print("Usage: python script.py <udp_ip> <udp_port> <min_distance> <max_distance> [rate] [constant]")
        sys.exit(1)

    # Parse command line arguments
    udp_ip = sys.argv[1]
    udp_port = int(sys.argv[2])
    min_distance = int(sys.argv[3])
    max_distance = int(sys.argv[4])
    
    # Set default rate to 1.0 if not provided
    rate = float(sys.argv[5]) if len(sys.argv) == 6 else 1.0

    # Set constant distance if provided
    constant = int(sys.argv[6]) if len(sys.argv) == 7 else None

    # Run the main function
    main(udp_ip, udp_port, min_distance, max_distance, rate, constant)