#pragma once
// MESSAGE AVSS_PRS_SYS_STATUS PACKING

#define MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS 60050


typedef struct __mavlink_avss_prs_sys_status_t {
 uint32_t error_status; /*<  PRS error statuses*/
 uint32_t time_boot_ms; /*<  Time since PRS system boot*/
 uint16_t battery_status; /*<  Estimated battery run-time without a remote connection and PRS battery voltage*/
 uint8_t arm_status; /*<  PRS arm statuses*/
 uint8_t change_status; /*<  PRS battery change statuses*/
} mavlink_avss_prs_sys_status_t;

#define MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS_LEN 12
#define MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS_MIN_LEN 12
#define MAVLINK_MSG_ID_60050_LEN 12
#define MAVLINK_MSG_ID_60050_MIN_LEN 12

#define MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS_CRC 153
#define MAVLINK_MSG_ID_60050_CRC 153



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_AVSS_PRS_SYS_STATUS { \
    60050, \
    "AVSS_PRS_SYS_STATUS", \
    5, \
    {  { "arm_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_avss_prs_sys_status_t, arm_status) }, \
         { "battery_status", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_avss_prs_sys_status_t, battery_status) }, \
         { "error_status", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_avss_prs_sys_status_t, error_status) }, \
         { "change_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_avss_prs_sys_status_t, change_status) }, \
         { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_avss_prs_sys_status_t, time_boot_ms) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_AVSS_PRS_SYS_STATUS { \
    "AVSS_PRS_SYS_STATUS", \
    5, \
    {  { "arm_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_avss_prs_sys_status_t, arm_status) }, \
         { "battery_status", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_avss_prs_sys_status_t, battery_status) }, \
         { "error_status", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_avss_prs_sys_status_t, error_status) }, \
         { "change_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_avss_prs_sys_status_t, change_status) }, \
         { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_avss_prs_sys_status_t, time_boot_ms) }, \
         } \
}
#endif

/**
 * @brief Pack a avss_prs_sys_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param arm_status  PRS arm statuses
 * @param battery_status  Estimated battery run-time without a remote connection and PRS battery voltage
 * @param error_status  PRS error statuses
 * @param change_status  PRS battery change statuses
 * @param time_boot_ms  Time since PRS system boot
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_avss_prs_sys_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t arm_status, uint16_t battery_status, uint32_t error_status, uint8_t change_status, uint32_t time_boot_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, error_status);
    _mav_put_uint32_t(buf, 4, time_boot_ms);
    _mav_put_uint16_t(buf, 8, battery_status);
    _mav_put_uint8_t(buf, 10, arm_status);
    _mav_put_uint8_t(buf, 11, change_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS_LEN);
#else
    mavlink_avss_prs_sys_status_t packet;
    packet.error_status = error_status;
    packet.time_boot_ms = time_boot_ms;
    packet.battery_status = battery_status;
    packet.arm_status = arm_status;
    packet.change_status = change_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS_MIN_LEN, MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS_LEN, MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS_CRC);
}

/**
 * @brief Pack a avss_prs_sys_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param arm_status  PRS arm statuses
 * @param battery_status  Estimated battery run-time without a remote connection and PRS battery voltage
 * @param error_status  PRS error statuses
 * @param change_status  PRS battery change statuses
 * @param time_boot_ms  Time since PRS system boot
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_avss_prs_sys_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t arm_status,uint16_t battery_status,uint32_t error_status,uint8_t change_status,uint32_t time_boot_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, error_status);
    _mav_put_uint32_t(buf, 4, time_boot_ms);
    _mav_put_uint16_t(buf, 8, battery_status);
    _mav_put_uint8_t(buf, 10, arm_status);
    _mav_put_uint8_t(buf, 11, change_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS_LEN);
#else
    mavlink_avss_prs_sys_status_t packet;
    packet.error_status = error_status;
    packet.time_boot_ms = time_boot_ms;
    packet.battery_status = battery_status;
    packet.arm_status = arm_status;
    packet.change_status = change_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS_MIN_LEN, MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS_LEN, MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS_CRC);
}

/**
 * @brief Encode a avss_prs_sys_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param avss_prs_sys_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_avss_prs_sys_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_avss_prs_sys_status_t* avss_prs_sys_status)
{
    return mavlink_msg_avss_prs_sys_status_pack(system_id, component_id, msg, avss_prs_sys_status->arm_status, avss_prs_sys_status->battery_status, avss_prs_sys_status->error_status, avss_prs_sys_status->change_status, avss_prs_sys_status->time_boot_ms);
}

/**
 * @brief Encode a avss_prs_sys_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param avss_prs_sys_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_avss_prs_sys_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_avss_prs_sys_status_t* avss_prs_sys_status)
{
    return mavlink_msg_avss_prs_sys_status_pack_chan(system_id, component_id, chan, msg, avss_prs_sys_status->arm_status, avss_prs_sys_status->battery_status, avss_prs_sys_status->error_status, avss_prs_sys_status->change_status, avss_prs_sys_status->time_boot_ms);
}

/**
 * @brief Send a avss_prs_sys_status message
 * @param chan MAVLink channel to send the message
 *
 * @param arm_status  PRS arm statuses
 * @param battery_status  Estimated battery run-time without a remote connection and PRS battery voltage
 * @param error_status  PRS error statuses
 * @param change_status  PRS battery change statuses
 * @param time_boot_ms  Time since PRS system boot
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_avss_prs_sys_status_send(mavlink_channel_t chan, uint8_t arm_status, uint16_t battery_status, uint32_t error_status, uint8_t change_status, uint32_t time_boot_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, error_status);
    _mav_put_uint32_t(buf, 4, time_boot_ms);
    _mav_put_uint16_t(buf, 8, battery_status);
    _mav_put_uint8_t(buf, 10, arm_status);
    _mav_put_uint8_t(buf, 11, change_status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS, buf, MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS_MIN_LEN, MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS_LEN, MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS_CRC);
#else
    mavlink_avss_prs_sys_status_t packet;
    packet.error_status = error_status;
    packet.time_boot_ms = time_boot_ms;
    packet.battery_status = battery_status;
    packet.arm_status = arm_status;
    packet.change_status = change_status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS, (const char *)&packet, MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS_MIN_LEN, MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS_LEN, MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS_CRC);
#endif
}

/**
 * @brief Send a avss_prs_sys_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_avss_prs_sys_status_send_struct(mavlink_channel_t chan, const mavlink_avss_prs_sys_status_t* avss_prs_sys_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_avss_prs_sys_status_send(chan, avss_prs_sys_status->arm_status, avss_prs_sys_status->battery_status, avss_prs_sys_status->error_status, avss_prs_sys_status->change_status, avss_prs_sys_status->time_boot_ms);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS, (const char *)avss_prs_sys_status, MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS_MIN_LEN, MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS_LEN, MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_avss_prs_sys_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t arm_status, uint16_t battery_status, uint32_t error_status, uint8_t change_status, uint32_t time_boot_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, error_status);
    _mav_put_uint32_t(buf, 4, time_boot_ms);
    _mav_put_uint16_t(buf, 8, battery_status);
    _mav_put_uint8_t(buf, 10, arm_status);
    _mav_put_uint8_t(buf, 11, change_status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS, buf, MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS_MIN_LEN, MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS_LEN, MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS_CRC);
#else
    mavlink_avss_prs_sys_status_t *packet = (mavlink_avss_prs_sys_status_t *)msgbuf;
    packet->error_status = error_status;
    packet->time_boot_ms = time_boot_ms;
    packet->battery_status = battery_status;
    packet->arm_status = arm_status;
    packet->change_status = change_status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS, (const char *)packet, MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS_MIN_LEN, MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS_LEN, MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE AVSS_PRS_SYS_STATUS UNPACKING


/**
 * @brief Get field arm_status from avss_prs_sys_status message
 *
 * @return  PRS arm statuses
 */
static inline uint8_t mavlink_msg_avss_prs_sys_status_get_arm_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field battery_status from avss_prs_sys_status message
 *
 * @return  Estimated battery run-time without a remote connection and PRS battery voltage
 */
static inline uint16_t mavlink_msg_avss_prs_sys_status_get_battery_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field error_status from avss_prs_sys_status message
 *
 * @return  PRS error statuses
 */
static inline uint32_t mavlink_msg_avss_prs_sys_status_get_error_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field change_status from avss_prs_sys_status message
 *
 * @return  PRS battery change statuses
 */
static inline uint8_t mavlink_msg_avss_prs_sys_status_get_change_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Get field time_boot_ms from avss_prs_sys_status message
 *
 * @return  Time since PRS system boot
 */
static inline uint32_t mavlink_msg_avss_prs_sys_status_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Decode a avss_prs_sys_status message into a struct
 *
 * @param msg The message to decode
 * @param avss_prs_sys_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_avss_prs_sys_status_decode(const mavlink_message_t* msg, mavlink_avss_prs_sys_status_t* avss_prs_sys_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    avss_prs_sys_status->error_status = mavlink_msg_avss_prs_sys_status_get_error_status(msg);
    avss_prs_sys_status->time_boot_ms = mavlink_msg_avss_prs_sys_status_get_time_boot_ms(msg);
    avss_prs_sys_status->battery_status = mavlink_msg_avss_prs_sys_status_get_battery_status(msg);
    avss_prs_sys_status->arm_status = mavlink_msg_avss_prs_sys_status_get_arm_status(msg);
    avss_prs_sys_status->change_status = mavlink_msg_avss_prs_sys_status_get_change_status(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS_LEN? msg->len : MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS_LEN;
        memset(avss_prs_sys_status, 0, MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS_LEN);
    memcpy(avss_prs_sys_status, _MAV_PAYLOAD(msg), len);
#endif
}
