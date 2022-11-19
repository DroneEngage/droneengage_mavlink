// MESSAGE AVSS_DRONE_POSITION support class

#pragma once

namespace mavlink {
namespace AVSSUAS {
namespace msg {

/**
 * @brief AVSS_DRONE_POSITION message
 *
 *  Drone position.
 */
struct AVSS_DRONE_POSITION : mavlink::Message {
    static constexpr msgid_t MSG_ID = 60051;
    static constexpr size_t LENGTH = 24;
    static constexpr size_t MIN_LENGTH = 24;
    static constexpr uint8_t CRC_EXTRA = 245;
    static constexpr auto NAME = "AVSS_DRONE_POSITION";


    uint32_t time_boot_ms; /*< [ms] Timestamp (time since FC boot). */
    int32_t lat; /*< [degE7] Latitude, expressed */
    int32_t lon; /*< [degE7] Longitude, expressed */
    int32_t alt; /*< [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL. */
    float ground_alt; /*< [m] Altitude above ground, This altitude is measured by a ultrasound, Laser rangefinder or millimeter-wave radar */
    float barometer_alt; /*< [m] This altitude is measured by a barometer */


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
        ss << "  lat: " << lat << std::endl;
        ss << "  lon: " << lon << std::endl;
        ss << "  alt: " << alt << std::endl;
        ss << "  ground_alt: " << ground_alt << std::endl;
        ss << "  barometer_alt: " << barometer_alt << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << time_boot_ms;                  // offset: 0
        map << lat;                           // offset: 4
        map << lon;                           // offset: 8
        map << alt;                           // offset: 12
        map << ground_alt;                    // offset: 16
        map << barometer_alt;                 // offset: 20
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> time_boot_ms;                  // offset: 0
        map >> lat;                           // offset: 4
        map >> lon;                           // offset: 8
        map >> alt;                           // offset: 12
        map >> ground_alt;                    // offset: 16
        map >> barometer_alt;                 // offset: 20
    }
};

} // namespace msg
} // namespace AVSSUAS
} // namespace mavlink
