/** @file
 *	@brief MAVLink comm testsuite protocol generated from AVSSUAS.xml
 *	@see http://mavlink.org
 */

#pragma once

#include <gtest/gtest.h>
#include "AVSSUAS.hpp"

#ifdef TEST_INTEROP
using namespace mavlink;
#undef MAVLINK_HELPER
#include "mavlink.h"
#endif


TEST(AVSSUAS, AVSS_PRS_SYS_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::AVSSUAS::msg::AVSS_PRS_SYS_STATUS packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.error_status = 963497672;
    packet_in.battery_status = 963497880;
    packet_in.arm_status = 41;
    packet_in.charge_status = 108;

    mavlink::AVSSUAS::msg::AVSS_PRS_SYS_STATUS packet1{};
    mavlink::AVSSUAS::msg::AVSS_PRS_SYS_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.error_status, packet2.error_status);
    EXPECT_EQ(packet1.battery_status, packet2.battery_status);
    EXPECT_EQ(packet1.arm_status, packet2.arm_status);
    EXPECT_EQ(packet1.charge_status, packet2.charge_status);
}

#ifdef TEST_INTEROP
TEST(AVSSUAS_interop, AVSS_PRS_SYS_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_avss_prs_sys_status_t packet_c {
         963497464, 963497672, 963497880, 41, 108
    };

    mavlink::AVSSUAS::msg::AVSS_PRS_SYS_STATUS packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.error_status = 963497672;
    packet_in.battery_status = 963497880;
    packet_in.arm_status = 41;
    packet_in.charge_status = 108;

    mavlink::AVSSUAS::msg::AVSS_PRS_SYS_STATUS packet2{};

    mavlink_msg_avss_prs_sys_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.error_status, packet2.error_status);
    EXPECT_EQ(packet_in.battery_status, packet2.battery_status);
    EXPECT_EQ(packet_in.arm_status, packet2.arm_status);
    EXPECT_EQ(packet_in.charge_status, packet2.charge_status);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(AVSSUAS, AVSS_DRONE_POSITION)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::AVSSUAS::msg::AVSS_DRONE_POSITION packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.lat = 963497672;
    packet_in.lon = 963497880;
    packet_in.alt = 963498088;
    packet_in.ground_alt = 129.0;
    packet_in.barometer_alt = 157.0;

    mavlink::AVSSUAS::msg::AVSS_DRONE_POSITION packet1{};
    mavlink::AVSSUAS::msg::AVSS_DRONE_POSITION packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.lat, packet2.lat);
    EXPECT_EQ(packet1.lon, packet2.lon);
    EXPECT_EQ(packet1.alt, packet2.alt);
    EXPECT_EQ(packet1.ground_alt, packet2.ground_alt);
    EXPECT_EQ(packet1.barometer_alt, packet2.barometer_alt);
}

#ifdef TEST_INTEROP
TEST(AVSSUAS_interop, AVSS_DRONE_POSITION)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_avss_drone_position_t packet_c {
         963497464, 963497672, 963497880, 963498088, 129.0, 157.0
    };

    mavlink::AVSSUAS::msg::AVSS_DRONE_POSITION packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.lat = 963497672;
    packet_in.lon = 963497880;
    packet_in.alt = 963498088;
    packet_in.ground_alt = 129.0;
    packet_in.barometer_alt = 157.0;

    mavlink::AVSSUAS::msg::AVSS_DRONE_POSITION packet2{};

    mavlink_msg_avss_drone_position_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.lat, packet2.lat);
    EXPECT_EQ(packet_in.lon, packet2.lon);
    EXPECT_EQ(packet_in.alt, packet2.alt);
    EXPECT_EQ(packet_in.ground_alt, packet2.ground_alt);
    EXPECT_EQ(packet_in.barometer_alt, packet2.barometer_alt);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(AVSSUAS, AVSS_DRONE_IMU)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::AVSSUAS::msg::AVSS_DRONE_IMU packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.q1 = 45.0;
    packet_in.q2 = 73.0;
    packet_in.q3 = 101.0;
    packet_in.q4 = 129.0;
    packet_in.xacc = 157.0;
    packet_in.yacc = 185.0;
    packet_in.zacc = 213.0;
    packet_in.xgyro = 241.0;
    packet_in.ygyro = 269.0;
    packet_in.zgyro = 297.0;

    mavlink::AVSSUAS::msg::AVSS_DRONE_IMU packet1{};
    mavlink::AVSSUAS::msg::AVSS_DRONE_IMU packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.q1, packet2.q1);
    EXPECT_EQ(packet1.q2, packet2.q2);
    EXPECT_EQ(packet1.q3, packet2.q3);
    EXPECT_EQ(packet1.q4, packet2.q4);
    EXPECT_EQ(packet1.xacc, packet2.xacc);
    EXPECT_EQ(packet1.yacc, packet2.yacc);
    EXPECT_EQ(packet1.zacc, packet2.zacc);
    EXPECT_EQ(packet1.xgyro, packet2.xgyro);
    EXPECT_EQ(packet1.ygyro, packet2.ygyro);
    EXPECT_EQ(packet1.zgyro, packet2.zgyro);
}

#ifdef TEST_INTEROP
TEST(AVSSUAS_interop, AVSS_DRONE_IMU)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_avss_drone_imu_t packet_c {
         963497464, 45.0, 73.0, 101.0, 129.0, 157.0, 185.0, 213.0, 241.0, 269.0, 297.0
    };

    mavlink::AVSSUAS::msg::AVSS_DRONE_IMU packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.q1 = 45.0;
    packet_in.q2 = 73.0;
    packet_in.q3 = 101.0;
    packet_in.q4 = 129.0;
    packet_in.xacc = 157.0;
    packet_in.yacc = 185.0;
    packet_in.zacc = 213.0;
    packet_in.xgyro = 241.0;
    packet_in.ygyro = 269.0;
    packet_in.zgyro = 297.0;

    mavlink::AVSSUAS::msg::AVSS_DRONE_IMU packet2{};

    mavlink_msg_avss_drone_imu_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.q1, packet2.q1);
    EXPECT_EQ(packet_in.q2, packet2.q2);
    EXPECT_EQ(packet_in.q3, packet2.q3);
    EXPECT_EQ(packet_in.q4, packet2.q4);
    EXPECT_EQ(packet_in.xacc, packet2.xacc);
    EXPECT_EQ(packet_in.yacc, packet2.yacc);
    EXPECT_EQ(packet_in.zacc, packet2.zacc);
    EXPECT_EQ(packet_in.xgyro, packet2.xgyro);
    EXPECT_EQ(packet_in.ygyro, packet2.ygyro);
    EXPECT_EQ(packet_in.zgyro, packet2.zgyro);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(AVSSUAS, AVSS_DRONE_OPERATION_MODE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::AVSSUAS::msg::AVSS_DRONE_OPERATION_MODE packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.M300_operation_mode = 17;
    packet_in.horsefly_operation_mode = 84;

    mavlink::AVSSUAS::msg::AVSS_DRONE_OPERATION_MODE packet1{};
    mavlink::AVSSUAS::msg::AVSS_DRONE_OPERATION_MODE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.M300_operation_mode, packet2.M300_operation_mode);
    EXPECT_EQ(packet1.horsefly_operation_mode, packet2.horsefly_operation_mode);
}

#ifdef TEST_INTEROP
TEST(AVSSUAS_interop, AVSS_DRONE_OPERATION_MODE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_avss_drone_operation_mode_t packet_c {
         963497464, 17, 84
    };

    mavlink::AVSSUAS::msg::AVSS_DRONE_OPERATION_MODE packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.M300_operation_mode = 17;
    packet_in.horsefly_operation_mode = 84;

    mavlink::AVSSUAS::msg::AVSS_DRONE_OPERATION_MODE packet2{};

    mavlink_msg_avss_drone_operation_mode_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.M300_operation_mode, packet2.M300_operation_mode);
    EXPECT_EQ(packet_in.horsefly_operation_mode, packet2.horsefly_operation_mode);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif
