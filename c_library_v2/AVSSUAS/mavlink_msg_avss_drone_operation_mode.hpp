// MESSAGE AVSS_DRONE_OPERATION_MODE support class

#pragma once

namespace mavlink {
namespace AVSSUAS {
namespace msg {

/**
 * @brief AVSS_DRONE_OPERATION_MODE message
 *
 *  Drone operation mode.
 */
struct AVSS_DRONE_OPERATION_MODE : mavlink::Message {
    static constexpr msgid_t MSG_ID = 60053;
    static constexpr size_t LENGTH = 6;
    static constexpr size_t MIN_LENGTH = 6;
    static constexpr uint8_t CRC_EXTRA = 45;
    static constexpr auto NAME = "AVSS_DRONE_OPERATION_MODE";


    uint32_t time_boot_ms; /*< [ms] Timestamp (time since FC boot). */
    uint8_t M300_operation_mode; /*<  DJI M300 operation mode */
    uint8_t horsefly_operation_mode; /*<  horsefly operation mode */


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
        ss << "  time_boot_ms: " << time_boot_ms << std::endl;
        ss << "  M300_operation_mode: " << +M300_operation_mode << std::endl;
        ss << "  horsefly_operation_mode: " << +horsefly_operation_mode << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << time_boot_ms;                  // offset: 0
        map << M300_operation_mode;           // offset: 4
        map << horsefly_operation_mode;       // offset: 5
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> time_boot_ms;                  // offset: 0
        map >> M300_operation_mode;           // offset: 4
        map >> horsefly_operation_mode;       // offset: 5
    }
};

} // namespace msg
} // namespace AVSSUAS
} // namespace mavlink
