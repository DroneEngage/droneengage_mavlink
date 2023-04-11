# Python Script to Send MAVLink Heartbeat Messages

This Python script uses the pymavlink library to send MAVLink heartbeat messages to a specified IP address and port. The script takes four command-line arguments: the IP address of the destination, the port number of the destination, the system ID, and the component ID.
Libraries Used

**This script uses the following libraries:
**
    pymavlink - for sending MAVLink heartbeat messages
    time - for adding a delay between sending messages
    argparse - for parsing command-line arguments

send_heartbeat Function

The send_heartbeat function is the main function of this script. It starts a connection listening on a UDP port using the mavutil.mavlink_connection() method. It then sends MAVLink heartbeat messages every 100ms using the the_connection.mav.heartbeat_send() method.

**The send_heartbeat function takes four arguments:
**
    ip_address - the IP address of the destination
    port - the port number of the destination
    system_id - the system ID of the sender
    component_id - the component ID of the sender

## main Function

The main function uses the argparse module to parse the command-line arguments. It then calls the send_heartbeat function with the parsed arguments.
Output

The script displays a message when a heartbeat is sent. The message shows the system ID and component ID of the sender, as well as the target system ID and IP address and port number of the destination.
Usage

To use this script, run it from the command line and provide the following arguments:

    ip - the IP address of the destination
    port - the port number of the destination
    system_id - the system ID of the sender
    component_id - the component ID of the sender

## Here's an example command:

python script.py 192.168.1.100 14550 1 1

This command sends MAVLink heartbeat messages to 192.168.1.100 on port 14550, with a sender system ID of 1 and component ID of 1.