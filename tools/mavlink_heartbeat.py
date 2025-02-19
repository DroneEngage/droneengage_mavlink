################################################################################
# 
# Author: M. Hefny  (mohammad.hefny@gmail.com)  
# Date: Oct 2024
# 
# Description:
# This script sends MAVLink HEARTBEAT messages to a specified destination 
# over a UDP connection. This can be used to maintain communication with 
# ground control software and resolve issues in the UI where heartbeat 
# signals may be needed.
#
# Usage:
# python send_heartbeat.py <ip> <port> <system_id> <component_id>
#
# Parameters:
# <ip>            : IP address of the destination (e.g., 127.0.0.1)
# <port>          : Port number of the destination (e.g., 14551)
# <system_id>     : System ID to be used in the heartbeat message
# <component_id>  : Component ID to be used in the heartbeat message
#
# Example:
# python send_heartbeat.py 127.0.0.1 14551 1 1
#
################################################################################

from pymavlink import mavutil
import time
import argparse

def send_heartbeat(ip_address, port, system_id, component_id):
    """
    Sends MAVLink HEARTBEAT messages to the specified IP address and port.

    Parameters:
    ip_address (str): The IP address of the destination.
    port (int): The port number of the destination.
    system_id (int): The system ID for the heartbeat message.
    component_id (int): The component ID for the heartbeat message.
    """
    # Start a connection listening on a UDP port
    the_connection = mavutil.mavlink_connection('udpout:{0}:{1}'.format(ip_address, port), source_system=system_id, source_component=component_id)

    # Display a message showing the source and target systems and ports
    print(f"Heartbeat from (SYSID {system_id}:{component_id}) to (SYSID {the_connection.target_system} @ {ip_address}:{port})")

    # Send MAVLink heartbeat messages every 100ms
    while True:
        ts = time.time()  # Capture the current timestamp
        time.sleep(100 / 1000)  # Sleep for 100 milliseconds
        # Send the heartbeat message
        the_connection.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0, 3  # Custom parameters for the heartbeat
        )

if __name__ == '__main__':
    # Parse command-line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('ip', help='IP address of the destination')
    parser.add_argument('port', type=int, help='Port number of the destination')
    parser.add_argument('system_id', type=int, help='System ID')
    parser.add_argument('component_id', type=int, help='Component ID')
    args = parser.parse_args()

    # Call the send_heartbeat function with the parsed arguments
    send_heartbeat(args.ip, args.port, args.system_id, args.component_id)
    