// MESSAGE AVSS_PRS_SYS_STATUS support class

#pragma once

namespace mavlink {
namespace AVSSUAS {
namespace msg {

/**
 * @brief AVSS_PRS_SYS_STATUS message
 *
 *  AVSS PRS system status.
 */
struct AVSS_PRS_SYS_STATUS : mavlink::Message {
    static constexpr msgid_t MSG_ID = 60050;
    static constexpr size_t LENGTH = 14;
    static constexpr size_t MIN_LENGTH = 14;
    static constexpr uint8_t CRC_EXTRA = 220;
    static constexpr auto NAME = "AVSS_PRS_SYS_STATUS";


    uint32_t time_boot_ms; /*< [ms] Timestamp (time since PRS boot). */
    uint32_t error_status; /*<  PRS error statuses */
    uint32_t battery_status; /*<  Estimated battery run-time without a remote connection and PRS battery voltage */
    uint8_t arm_status; /*<  PRS arm statuses */
    uint8_t charge_status; /*<  PRS battery charge statuses */


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
        ss << "  error_status: " << error_status << std::endl;
        ss << "  battery_status: " << battery_status << std::endl;
        ss << "  arm_status: " << +arm_status << std::endl;
        ss << "  charge_status: " << +charge_status << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << time_boot_ms;                  // offset: 0
        map << error_status;                  // offset: 4
        map << battery_status;                // offset: 8
        map << arm_status;                    // offset: 12
        map << charge_status;                 // offset: 13
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> time_boot_ms;                  // offset: 0
        map >> error_status;                  // offset: 4
        map >> battery_status;                // offset: 8
        map >> arm_status;                    // offset: 12
        map >> charge_status;                 // offset: 13
    }
};

} // namespace msg
} // namespace AVSSUAS
} // namespace mavlink
