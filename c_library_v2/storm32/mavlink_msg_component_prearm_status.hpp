// MESSAGE COMPONENT_PREARM_STATUS support class

#pragma once

namespace mavlink {
namespace storm32 {
namespace msg {

/**
 * @brief COMPONENT_PREARM_STATUS message
 *
 * Message reporting the status of the prearm checks. The flags are component specific.
 */
struct COMPONENT_PREARM_STATUS : mavlink::Message {
    static constexpr msgid_t MSG_ID = 60025;
    static constexpr size_t LENGTH = 10;
    static constexpr size_t MIN_LENGTH = 10;
    static constexpr uint8_t CRC_EXTRA = 20;
    static constexpr auto NAME = "COMPONENT_PREARM_STATUS";


    uint8_t target_system; /*<  System ID */
    uint8_t target_component; /*<  Component ID */
    uint32_t enabled_flags; /*<  Currently enabled prearm checks. 0 means no checks are being performed, UINT32_MAX means not known. */
    uint32_t fail_flags; /*<  Currently not passed prearm checks. 0 means all checks have been passed. */


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
        ss << "  target_system: " << +target_system << std::endl;
        ss << "  target_component: " << +target_component << std::endl;
        ss << "  enabled_flags: " << enabled_flags << std::endl;
        ss << "  fail_flags: " << fail_flags << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << enabled_flags;                 // offset: 0
        map << fail_flags;                    // offset: 4
        map << target_system;                 // offset: 8
        map << target_component;              // offset: 9
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> enabled_flags;                 // offset: 0
        map >> fail_flags;                    // offset: 4
        map >> target_system;                 // offset: 8
        map >> target_component;              // offset: 9
    }
};

} // namespace msg
} // namespace storm32
} // namespace mavlink
