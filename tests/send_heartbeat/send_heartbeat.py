# This function is used to test sending a HEARTBEAT next to HEARTBEAT sent by FC.
# This is used to solve some issues in UI


from pymavlink import mavutil
import time
import argparse




def send_heartbeat(ip_address, port, system_id, component_id):
    # Start a connection listening on a UDP port
    the_connection = mavutil.mavlink_connection('udpout:{0}:{1}'.format(ip_address, port), source_system=system_id, source_component=component_id)

    # Display a message showing the source and target systems and ports
    print(f"Heartbeat from (SYSID {system_id}:{component_id}) to (SYSID {the_connection.target_system} @ {ip_address}:{port})")

    # Send MAVLink heartbeat messages every 100ms
    while True:
        ts = time.time()
        time.sleep(100/1000)
        the_connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                          mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                          0, 0, 0, 3)

if __name__ == '__main__':
    # Parse command-line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('ip', help='IP address of the destination')
    parser.add_argument('port', type=int, help='port number of the destination')
    parser.add_argument('system_id', type=int, help='system ID')
    parser.add_argument('component_id', type=int, help='component ID')
    args = parser.parse_args()

    # Call the send_heartbeat function with the parsed arguments
    send_heartbeat(args.ip, args.port, args.system_id, args.component_id)
    