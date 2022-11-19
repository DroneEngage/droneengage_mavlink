// MESSAGE MISSION_CHECKSUM support class

#pragma once

namespace mavlink {
namespace development {
namespace msg {

/**
 * @brief MISSION_CHECKSUM message
 *
 * Checksum for the current mission, rally point or geofence plan, or for the "combined" plan (a GCS can use these checksums to determine if it has matching plans).
        This message must be broadcast with the appropriate checksum following any change to a mission, geofence or rally point definition
        (immediately after the MISSION_ACK that completes the upload sequence).
        It may also be requested using MAV_CMD_REQUEST_MESSAGE, where param 2 indicates the plan type for which the checksum is required.
        The checksum must be calculated on the autopilot, but may also be calculated by the GCS.
        The checksum uses the same CRC32 algorithm as MAVLink FTP (https://mavlink.io/en/services/ftp.html#crc32-implementation).
        The checksum for a mission, geofence or rally point definition is run over each item in the plan in seq order (excluding the home location if present in the plan), and covers the following fields (in order):
        frame, command, autocontinue, param1, param2, param3, param4, param5, param6, param7.
        The checksum for the whole plan (MAV_MISSION_TYPE_ALL) is calculated using the same approach, running over each sub-plan in the following order: mission, geofence then rally point.
      
 */
struct MISSION_CHECKSUM : mavlink::Message {
    static constexpr msgid_t MSG_ID = 53;
    static constexpr size_t LENGTH = 5;
    static constexpr size_t MIN_LENGTH = 5;
    static constexpr uint8_t CRC_EXTRA = 3;
    static constexpr auto NAME = "MISSION_CHECKSUM";


    uint8_t mission_type; /*<  Mission type. */
    uint32_t checksum; /*<  CRC32 checksum of current plan for specified type. */


    inline std::string get_name(void) const override
    {
            return NAME;
    }

    inline Info get_message_info(void) const override
    {
            return { MSG_ID, LENGTH, MIN_LENGTH, CRC_EXTRA };
    }

    inline std::string to_yaml(void) const override
    {
        std::stringstream ss;

        ss << NAME << ":" << std::endl;
        ss << "  mission_type: " << +mission_type << std::endl;
        ss << "  checksum: " << checksum << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << checksum;                      // offset: 0
        map << mission_type;                  // offset: 4
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> checksum;                      // offset: 0
        map >> mission_type;                  // offset: 4
    }
};

} // namespace msg
} // namespace development
} // namespace mavlink
