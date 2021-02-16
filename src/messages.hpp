

// InterModules command
#define CMD_TYPE_INTERMODULE "uv"








// Communication Commands
// group broadcast
#define CMD_COMM_GROUP                  "g" 
// individual broadcast
#define CMD_COMM_INDIVIDUAL             "i" 
    

#define ANDRUAV_PROTOCOL_GROUP_ID       "gr"
#define ANDRUAV_PROTOCOL_SENDER         "sd"
#define ANDRUAV_PROTOCOL_COMM_TYPE      "cm"
#define ANDRUAV_PROTOCOL_TARGET_ID      "tg"
#define ANDRUAV_PROTOCOL_MESSAGE_TYPE   "mt"
#define ANDRUAV_PROTOCOL_MESSAGE_CMD    "ms"
#define INTERMODULE_COMMAND_TYPE        "ty"


#define TYPE_AndruavModule_ID                   9100

#define TYPE_AndruavResala_GPS                  1002
#define TYPE_AndruavResala_POWER                1003
#define TYPE_AndruavResala_ID 	                1004
#define TYPE_AndruavResala_RemoteExecute 		1005     
#define TYPE_AndruavResala_Error                1008    
#define TYPE_AndruavResala_FlightControl        1010
#define TYPE_AndruavMessage_Arm                 1030
#define TYPE_AndruavResala_ChangeAltitude       1031
#define TYPE_AndruavResala_Land                 1032
#define TYPE_AndruavResala_NAV_INFO             1036


#define GPS_MODE_AUTO                           0
// .a.k.a mobile... i.e. gps info used bu uavos comm is not from the board
#define GPS_MODE_EXTERNAL                       1
#define GPS_MODE_FCB                            2