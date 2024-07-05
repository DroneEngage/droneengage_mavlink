

// InterModules command
/**
 * @brief CMD_TYPE_INTERMODULE is used when you want to send a message from a  module to another.
 * This is mainly used to emulate a message comes from an external gcs to a module but is created by another module.
 * i.e. FCB module can emulate take_image even comes from gcs to camera module.
 * Even if you do not use CMD_TYPE_INTERMODULE and uses a command id that is for inter-module commands such as id > 9500 
 * then it will be handled by communicator module such as TYPE_AndruavModule_RemoteExecute.
 */
#define CMD_TYPE_INTERMODULE "uv"
#define CMD_TYPE_SYSTEM_MSG  "s"

// JSON InterModule Fields
#define JSON_INTERMODULE_MODULE_ID              "a"
#define JSON_INTERMODULE_MODULE_CLASS           "b"
#define JSON_INTERMODULE_MODULE_MESSAGES_LIST   "c"
#define JSON_INTERMODULE_MODULE_FEATURES        "d"
#define JSON_INTERMODULE_MODULE_KEY             "e"
#define JSON_INTERMODULE_PARTY_RECORD           "f"
#define JSON_INTERMODULE_HARDWARE_ID            "s"
#define JSON_INTERMODULE_HARDWARE_TYPE          "t"
#define JSON_INTERMODULE_VERSION                "v"
#define JSON_INTERMODULE_TIMESTAMP_INSTANCE     "u"
#define JSON_INTERMODULE_RESEND                 "z"









// Communication Commands
/**
 * @brief Group boradcast
 * @details group broad cast overrides individual. 
 * @see Andruav_Communication_Server for details.
 */
#define CMD_COMM_GROUP                  "g" 
/**
 * @brief Individual broadcast.
 * @details single target except for the following
 * *_GD_* all GCS
 * *_AGN_* all agents
 * @see Andruav_Communication_Server for details.
 */
#define CMD_COMM_INDIVIDUAL             "i" 
    
/**
 * @brief System command.
 * @details this should be handled by communication server. e.g. task access messages.
 * @see Andruav_Communication_Server for details.
 */
#define CMD_COMM_SYSTEM                 "s" 
    



// Andruav Protocol Fields
#define ANDRUAV_PROTOCOL_GROUP_ID               "gr"
#define ANDRUAV_PROTOCOL_SENDER                 "sd"
#define ANDRUAV_PROTOCOL_TARGET_ID              "tg"
#define ANDRUAV_PROTOCOL_MESSAGE_TYPE           "mt"
#define ANDRUAV_PROTOCOL_MESSAGE_CMD            "ms"
#define ANDRUAV_PROTOCOL_MESSAGE_PERMISSION     "p"
#define INTERMODULE_ROUTING_TYPE                "ty"
#define INTERMODULE_MODULE_KEY                  "GU"

// Reserved Target Values
#define ANDRUAV_PROTOCOL_SENDER_ALL_GCS         "_GCS_"
#define ANDRUAV_PROTOCOL_SENDER_ALL_AGENTS      "_AGN_"
#define ANDRUAV_PROTOCOL_SENDER_ALL             "_GD_"
#define SPECIAL_NAME_SYS_NAME                   "_SYS_"


// SOCKET STATUS
#define SOCKET_STATUS_FREASH 			1   // socket is new
#define SOCKET_STATUS_CONNECTING    	2	// connecting to WS
#define SOCKET_STATUS_DISCONNECTING 	3   // disconnecting from WS
#define SOCKET_STATUS_DISCONNECTED 		4   // disconnected  from WS
#define SOCKET_STATUS_CONNECTED 		5   // connected to WS
#define SOCKET_STATUS_REGISTERED 		6   // connected and executed AddMe
#define SOCKET_STATUS_UNREGISTERED 		7   // connected but not registred
#define SOCKET_STATUS_ERROR 		    8   // Error




// System Messages
#define TYPE_AndruavSystem_LoadTasks		    9001
#define TYPE_AndruavSystem_SaveTasks		    9002
#define TYPE_AndruavSystem_DeleteTasks	        9003
#define TYPE_AndruavSystem_DisableTasks	        9004
#define TYPE_AndruavSystem_Ping                 9005
#define TYPE_AndruavSystem_LogoutCommServer     9006
#define TYPE_AndruavSystem_ConnectedCommServer  9007
#define TYPE_AndruavSystem_UDPProxy             9008

// Inter Module Commands
#define TYPE_AndruavModule_ID                   9100
#define TYPE_AndruavModule_RemoteExecute        9101
#define TYPE_AndruavModule_Location_Info        9102



// Andruav Messages
#define TYPE_AndruavMessage_GPS                         1002
#define TYPE_AndruavMessage_POWER                       1003
#define TYPE_AndruavMessage_ID 	                        1004
#define TYPE_AndruavMessage_RemoteExecute 		        1005     
#define TYPE_AndruavMessage_IMG                         1006     
#define TYPE_AndruavMessage_Error                       1008    
#define TYPE_AndruavMessage_FlightControl               1010
#define TYPE_AndruavMessage_DroneReport                 1020
#define TYPE_AndruavMessage_Signaling                   1021
#define TYPE_AndruavMessage_HomeLocation                1022
#define TYPE_AndruavMessage_GeoFence                    1023
#define TYPE_AndruavMessage_ExternalGeoFence            1024
#define TYPE_AndruavMessage_GEOFenceHit                 1025
#define TYPE_AndruavMessage_WayPoints                   1027
#define TYPE_AndruavMessage_GeoFenceAttachStatus        1029
#define TYPE_AndruavMessage_Arm                         1030
#define TYPE_AndruavMessage_ChangeAltitude              1031
#define TYPE_AndruavMessage_Land                        1032
#define TYPE_AndruavMessage_GuidedPoint                 1033
#define TYPE_AndruavMessage_CirclePoint                 1034
#define TYPE_AndruavMessage_DoYAW                       1035
#define TYPE_AndruavMessage_NAV_INFO                    1036
#define TYPE_AndruavMessage_DistinationLocation         1037
#define TYPE_AndruavMessage_ChangeSpeed                 1040
#define TYPE_AndruavMessage_Ctrl_Cameras                1041
#define TYPE_AndruavMessage_TrackingTarget              1042
#define TYPE_AndruavMessage_TrackingTargetLocation      1043
#define TYPE_AndruavMessage_TargetLost                  1044
#define TYPE_AndruavMessage_UploadWayPoints             1046
#define TYPE_AndruavMessage_RemoteControlSettings	    1047
#define TYPE_AndruavMessage_SET_HOME_LOCATION           1048
#define TYPE_AndruavMessage_RemoteControl2		        1052
/**
 * @brief tell a drone that another drone is in its team -a follower-.
 * @details 
 * This message can be sent from GCS or another Drone either a leader or not.
 * This message requests from the receiver "Drone" to send @ref TYPE_AndruavMessage_UpdateSwarm
 * to a third drone that is a LEADER requesting to join its swarm.
 * The receiver can refuse to send @ref TYPE_AndruavMessage_UpdateSwarm
 * and the third drone can also refuse the request to be followed by the receiver.
 * 
 * @note receiver should not assume it is a follower. It only should forward this request to the leader.
 */
#define TYPE_AndruavMessage_FollowHim_Request           1054
/**
 * @brief This message is sent from Leader drone to a follower. It guides it to the destination point that it wants it to go to.
 * @details
 * There is nothing called a Follower Drone
 * All Drones Obey AndruavResala_FollowMe_Guided EVEN if they are Leaders.<br>
 * If a Drone wants to IGNORE these messages that is OK for whatever reason.<br>
 * If a Drone wants to Stop others from sending such messages it can send ANdruavResala_UpdateSwarm with remove action.
 */
#define TYPE_AndruavMessage_FollowMe_Guided             1055
/**
 * @brief This command is sent to instruct a drone to be a leader with a swarm-formation.
 * A Formation FORMATION_SERB_NO_SWARM means there is no swarm mode anymore. 
 */
#define TYPE_AndruavMessage_MAKE_SWARM                  1056
#define TYPE_AndruavMessage_SwarmReport                 1057
/**
 * @brief This message is sent to Leader Drone to add a slave drone in a swarm and in an index.
 * given index may contradict with other indices. It is upto Leader Drone to handle this conflict.
 */
#define TYPE_AndruavMessage_UpdateSwarm                 1058
#define TYPE_AndruavMessage_Sync_EventFire              1061
#define TYPE_AndruavMessage_Prepherials                 1070
#define TYPE_AndruavMessage_UDPProxy_Info               1071
/**
 * Used to set unit name and description.
 * This message is mainly sent from web and received by communication module.
*/
#define TYPE_AndruavMessage_Unit_Name                   1072

// Binary Messages 

//deprecated telemetry technology
#define TYPE_AndruavMessage_LightTelemetry              2022

// New JSON Messages 
#define TYPE_AndruavMessage_ServoChannel                6001


// New Binary Messages 
#define TYPE_AndruavMessage_ServoOutput                 6501
#define TYPE_AndruavMessage_MAVLINK                     6502
#define TYPE_AndruavMessage_SWARM_MAVLINK               6503

/**
 * Used by other modules to exchange mavlink information
 * between each other.
 * This allows custom implementation for sharing mavlink info 
 * between mavlink module and other modules.
*/
#define TYPE_AndruavMessage_INTERNAL_MAVLINK            6504

#define TYPE_AndruavMessage_P2P_ACTION                  6505
#define TYPE_AndruavMessage_P2P_STATUS                  6506


#define TYPE_AndruavMessage_DUMMY                       9999
#define TYPE_AndruavMessage_USER_RANGE_START            80000
#define TYPE_AndruavMessage_USER_RANGE_END              90000

// Andruav Mission Types
#define TYPE_CMissionItem                                    0
#define TYPE_CMissionAction_Spline                           6
#define TYPE_CMissionItem_WayPointStep                      16 // same as mavlink
#define TYPE_CMissionAction_Circle                          18 // same as mavlink MAV_CMD_NAV_LOITER_TURNS
#define TYPE_CMissionAction_RTL                             20 // same as mavlink
#define TYPE_CMissionAction_Landing                         21 // same as mavlink
#define TYPE_CMissionAction_TakeOff                         22 // same as mavlink
#define TYPE_CMissionAction_CONTINUE_AND_CHANGE_ALT         30 // same as mavlink  
#define TYPE_CMissionAction_Guided_Enabled                  92 // same as mavlink
#define TYPE_CMissionAction_Delay                           93 // same as mavlink 
#define TYPE_CMissionAction_Delay_STATE_MACHINE            112 // same as mavlink
#define TYPE_CMissionAction_ChangeAlt                      113 // same as mavlink   
#define TYPE_CMissionAction_ChangeHeading                  115 // same as mavlink 
#define TYPE_CMissionAction_ChangeSpeed                    178 // same as mavlink
#define TYPE_CMissionAction_CameraControl                  203 // same as mavlink
#define TYPE_CMissionAction_CameraTrigger                  206 // same as mavlink
#define TYPE_CMissionAction_DummyMission                 99999


// P2P Parameters

#define P2P_ACTION_RESTART_TO_MAC                           0
#define P2P_ACTION_CONNECT_TO_MAC                           1
#define P2P_ACTION_CANDICATE_MAC                            2
#define P2P_ACTION_SCAN_NETWORK                             3
/**
 * @brief this is different from P2P_ACTION_CONNECT_TO_MAC 
 * in that it does not require direct access 
 * or specifies who is parent to whom.
 */
#define P2P_ACTION_ACCESS_TO_MAC                            4

#define P2P_STATUS_CONNECTED_TO_MAC                         0
#define P2P_STATUS_DISCONNECTED_FROM_MAC                    1



// CAMERA MODULE MESSAGES

#define EXTERNAL_CAMERA_TYPE_UNKNOWN                        0
#define EXTERNAL_CAMERA_TYPE_RTCWEBCAM                      2

#define RemoteCommand_STREAMVIDEO 		                    110
#define RemoteCommand_RECORDVIDEO 		                    111
#define RemoteCommand_STREAMVIDEORESUME 	                112
#define RemoteCommand_SWITCHCAM 			                114


// // Remote Control Sub Actions
// #define RC_SUB_ACTION_RELEASED                           0
// #define RC_SUB_ACTION_CENTER_CHANNELS                    1
// #define RC_SUB_ACTION_FREEZE_CHANNELS                    2
// #define RC_SUB_ACTION_JOYSTICK_CHANNELS                  4
// #define RC_SUB_ACTION_JOYSTICK_CHANNELS_GUIDED           8

// Remote Execute Commands
#define RemoteCommand_TELEMETRYCTRL                         108 // Telemetry streaming
#define RemoteCommand_STREAMVIDEO                           110
#define RemoteCommand_CONNECT_FCB                           118
#define RemoteCommand_GET_WAY_POINTS                        500 // get from andruav not FCB but you can still read from fcb and refresh all   
#define RemoteCommand_RELOAD_WAY_POINTS_FROM_FCB            501
#define RemoteCommand_CLEAR_WAY_POINTS_FROM_FCB             502
#define RemoteCommand_CLEAR_WAY_POINTS_FROM_FCB             502
#define RemoteCommand_CLEAR_FENCE_DATA 	                    503 // andruav fence
#define RemoteCommand_SET_START_MISSION_ITEM                504
#define RemoteCommand_REQUEST_PARA_LIST                     505 // list of FCB parameters
#define RemoteCommand_SET_UDPPROXY_CLIENT_PORT              506
#define RemoteCommand_MISSION_COUNT                         507
#define RemoteCommand_MISSION_CURRENT                       508


// Drone Report
#define Drone_Report_NAV_ItemReached            1

// Error Numbers
#define ERROR_TYPE_LO7ETTA7AKOM                 5
#define ERROR_3DR                               7
#define ERROR_GPS                               10
#define ERROR_POWER                             11
#define ERROR_RCCONTROL                         12
#define ERROR_GEO_FENCE_ERROR                   100

// 0	MAV_SEVERITY_EMERGENCY	System is unusable. This is a "panic" condition.
#define NOTIFICATION_TYPE_EMERGENCY             0
// 1	MAV_SEVERITY_ALERT	Action should be taken immediately. Indicates error in non-critical systems.
#define NOTIFICATION_TYPE_ALERT                 1
// 2	MAV_SEVERITY_CRITICAL	Action must be taken immediately. Indicates failure in a primary system.
#define NOTIFICATION_TYPE_CRITICAL              2
// 3	MAV_SEVERITY_ERROR	Indicates an error in secondary/redundant systems.
#define NOTIFICATION_TYPE_ERROR                 3
// 4	MAV_SEVERITY_WARNING	Indicates about a possible future error if this is not resolved within a given timeframe. Example would be a low battery warning.
#define NOTIFICATION_TYPE_WARNING               4
// 5	MAV_SEVERITY_NOTICE	An unusual event has occurred, though not an error condition. This should be investigated for the root cause.
#define NOTIFICATION_TYPE_NOTICE                5
// 6	MAV_SEVERITY_INFO	Normal operational messages. Useful for logging. No action is required for these messages.
#define NOTIFICATION_TYPE_INFO                  6
// 7	MAV_SEVERITY_DEBUG	Useful non-operational messages that can assist in debugging. These should not occur during normal operation.
#define NOTIFICATION_TYPE_DEBUG                 7







#define GPS_MODE_AUTO                           0
// .a.k.a mobile... i.e. gps info used bu de comm is not from the board
#define GPS_MODE_EXTERNAL                       1
#define GPS_MODE_FCB                            2



#define WAYPOINT_NO_CHUNK                       0
#define WAYPOINT_CHUNK                          1
#define WAYPOINT_LAST_CHUNK                     999


// Telemetry Request Remote Execute
#define CONST_TELEMETRY_REQUEST_START		1
#define CONST_TELEMETRY_REQUEST_END			2
#define CONST_TELEMETRY_REQUEST_RESUME		3
#define CONST_TELEMETRY_ADJUST_RATE		    4
#define CONST_TELEMETRY_REQUEST_PAUSE       5


// Fence Soft & Hard actions
#define CONST_FENCE_ACTION_SOFT                   0
#define CONST_FENCE_ACTION_RTL                    2
#define CONST_FENCE_ACTION_LAND                  12
#define CONST_FENCE_ACTION_LOITER                10
#define CONST_FENCE_ACTION_BRAKE                 17
#define CONST_FENCE_ACTION_SMART_RTL             21


// TYPE_AndruavMessage_UpdateSwarm actions
#define SWARM_UPDATED                               1
#define SWARM_DELETE                                2
// TYPE_AndruavMessage_FollowHim_Request actions
#define SWARM_FOLLOW                                1
#define SWARM_UNFOLLOW                              2

#define TASHKEEL_SERB_NO_SWARM                      0
#define TASHKEEL_SERB_THREAD                        1
#define TASHKEEL_SERB_VECTOR                        2
#define TASHKEEL_SERB_VECTOR_180                    3


// GCS Permissions
#define PERMISSION_ALLOW_GCS                0x00000001
#define PERMISSION_ALLOW_UNIT               0x00000010
#define PERMISSION_ALLOW_GCS_FULL_CONTROL   0x00000f00
#define PERMISSION_ALLOW_GCS_WP_CONTROL     0x00000100
#define PERMISSION_ALLOW_GCS_MODES_CONTROL  0x00000200
#define PERMISSION_ALLOW_GCS_MODES_SERVOS   0x00000400
#define PERMISSION_ALLOW_GCS_VIDEO          0x0000f000
#define PERMISSION_ALLOW_SWARM              0x000f0000


// DistinationLocation Types
#define DESTINATION_GUIDED_POINT            0
#define DESTINATION_SWARM_MY_LOCATION       1