/** @file
 *	@brief MAVLink comm testsuite protocol generated from development.xml
 *	@see http://mavlink.org
 */

#pragma once

#include <gtest/gtest.h>
#include "development.hpp"

#ifdef TEST_INTEROP
using namespace mavlink;
#undef MAVLINK_HELPER
#include "mavlink.h"
#endif


TEST(development, MISSION_CHECKSUM)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::development::msg::MISSION_CHECKSUM packet_in{};
    packet_in.mission_type = 17;
    packet_in.checksum = 963497464;

    mavlink::development::msg::MISSION_CHECKSUM packet1{};
    mavlink::development::msg::MISSION_CHECKSUM packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.mission_type, packet2.mission_type);
    EXPECT_EQ(packet1.checksum, packet2.checksum);
}

#ifdef TEST_INTEROP
TEST(development_interop, MISSION_CHECKSUM)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_mission_checksum_t packet_c {
         963497464, 17
    };

    mavlink::development::msg::MISSION_CHECKSUM packet_in{};
    packet_in.mission_type = 17;
    packet_in.checksum = 963497464;

    mavlink::development::msg::MISSION_CHECKSUM packet2{};

    mavlink_msg_mission_checksum_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.mission_type, packet2.mission_type);
    EXPECT_EQ(packet_in.checksum, packet2.checksum);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif
