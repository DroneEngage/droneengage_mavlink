// MESSAGE AVSS_DRONE_IMU support class

#pragma once

namespace mavlink {
namespace AVSSUAS {
namespace msg {

/**
 * @brief AVSS_DRONE_IMU message
 *
 *  Drone IMU data. Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0).
 */
struct AVSS_DRONE_IMU : mavlink::Message {
    static constexpr msgid_t MSG_ID = 60052;
    static constexpr size_t LENGTH = 44;
    static constexpr size_t MIN_LENGTH = 44;
    static constexpr uint8_t CRC_EXTRA = 101;
    static constexpr auto NAME = "AVSS_DRONE_IMU";


    uint32_t time_boot_ms; /*< [ms] Timestamp (time since FC boot). */
    float q1; /*<  Quaternion component 1, w (1 in null-rotation) */
    float q2; /*<  Quaternion component 2, x (0 in null-rotation) */
    float q3; /*<  Quaternion component 3, y (0 in null-rotation) */
    float q4; /*<  Quaternion component 4, z (0 in null-rotation) */
    float xacc; /*< [m/s/s] X acceleration */
    float yacc; /*< [m/s/s] Y acceleration */
    float zacc; /*< [m/s/s] Z acceleration */
    float xgyro; /*< [rad/s] Angular speed around X axis */
    float ygyro; /*< [rad/s] Angular speed around Y axis */
    float zgyro; /*< [rad/s] Angular speed around Z axis */


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
        ss << "  q1: " << q1 << std::endl;
        ss << "  q2: " << q2 << std::endl;
        ss << "  q3: " << q3 << std::endl;
        ss << "  q4: " << q4 << std::endl;
        ss << "  xacc: " << xacc << std::endl;
        ss << "  yacc: " << yacc << std::endl;
        ss << "  zacc: " << zacc << std::endl;
        ss << "  xgyro: " << xgyro << std::endl;
        ss << "  ygyro: " << ygyro << std::endl;
        ss << "  zgyro: " << zgyro << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << time_boot_ms;                  // offset: 0
        map << q1;                            // offset: 4
        map << q2;                            // offset: 8
        map << q3;                            // offset: 12
        map << q4;                            // offset: 16
        map << xacc;                          // offset: 20
        map << yacc;                          // offset: 24
        map << zacc;                          // offset: 28
        map << xgyro;                         // offset: 32
        map << ygyro;                         // offset: 36
        map << zgyro;                         // offset: 40
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> time_boot_ms;                  // offset: 0
        map >> q1;                            // offset: 4
        map >> q2;                            // offset: 8
        map >> q3;                            // offset: 12
        map >> q4;                            // offset: 16
        map >> xacc;                          // offset: 20
        map >> yacc;                          // offset: 24
        map >> zacc;                          // offset: 28
        map >> xgyro;                         // offset: 32
        map >> ygyro;                         // offset: 36
        map >> zgyro;                         // offset: 40
    }
};

} // namespace msg
} // namespace AVSSUAS
} // namespace mavlink
