#pragma once
// MESSAGE SCANNING_SONAR PACKING

#define MAVLINK_MSG_ID_SCANNING_SONAR 1000

MAVPACKED(
typedef struct __mavlink_scanning_sonar_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 uint16_t range; /*< [mm] Measured range*/
 uint16_t angle; /*< [0.1 degrees] Angle*/
 uint16_t roll; /*< [0.1 deg] Roll*/
 uint16_t pitch; /*< [0.1 deg] Pitch.*/
 uint16_t yaw; /*< [0.1 deg] Heading.*/
}) mavlink_scanning_sonar_t;

#define MAVLINK_MSG_ID_SCANNING_SONAR_LEN 14
#define MAVLINK_MSG_ID_SCANNING_SONAR_MIN_LEN 14
#define MAVLINK_MSG_ID_1000_LEN 14
#define MAVLINK_MSG_ID_1000_MIN_LEN 14

#define MAVLINK_MSG_ID_SCANNING_SONAR_CRC 117
#define MAVLINK_MSG_ID_1000_CRC 117



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SCANNING_SONAR { \
    1000, \
    "SCANNING_SONAR", \
    6, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_scanning_sonar_t, time_boot_ms) }, \
         { "range", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_scanning_sonar_t, range) }, \
         { "angle", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_scanning_sonar_t, angle) }, \
         { "roll", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_scanning_sonar_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_scanning_sonar_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_scanning_sonar_t, yaw) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SCANNING_SONAR { \
    "SCANNING_SONAR", \
    6, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_scanning_sonar_t, time_boot_ms) }, \
         { "range", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_scanning_sonar_t, range) }, \
         { "angle", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_scanning_sonar_t, angle) }, \
         { "roll", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_scanning_sonar_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_scanning_sonar_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_scanning_sonar_t, yaw) }, \
         } \
}
#endif

/**
 * @brief Pack a scanning_sonar message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param range [mm] Measured range
 * @param angle [0.1 degrees] Angle
 * @param roll [0.1 deg] Roll
 * @param pitch [0.1 deg] Pitch.
 * @param yaw [0.1 deg] Heading.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_scanning_sonar_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint16_t range, uint16_t angle, uint16_t roll, uint16_t pitch, uint16_t yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SCANNING_SONAR_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint16_t(buf, 4, range);
    _mav_put_uint16_t(buf, 6, angle);
    _mav_put_uint16_t(buf, 8, roll);
    _mav_put_uint16_t(buf, 10, pitch);
    _mav_put_uint16_t(buf, 12, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SCANNING_SONAR_LEN);
#else
    mavlink_scanning_sonar_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.range = range;
    packet.angle = angle;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SCANNING_SONAR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SCANNING_SONAR;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SCANNING_SONAR_MIN_LEN, MAVLINK_MSG_ID_SCANNING_SONAR_LEN, MAVLINK_MSG_ID_SCANNING_SONAR_CRC);
}

/**
 * @brief Pack a scanning_sonar message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param range [mm] Measured range
 * @param angle [0.1 degrees] Angle
 * @param roll [0.1 deg] Roll
 * @param pitch [0.1 deg] Pitch.
 * @param yaw [0.1 deg] Heading.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_scanning_sonar_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,uint16_t range,uint16_t angle,uint16_t roll,uint16_t pitch,uint16_t yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SCANNING_SONAR_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint16_t(buf, 4, range);
    _mav_put_uint16_t(buf, 6, angle);
    _mav_put_uint16_t(buf, 8, roll);
    _mav_put_uint16_t(buf, 10, pitch);
    _mav_put_uint16_t(buf, 12, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SCANNING_SONAR_LEN);
#else
    mavlink_scanning_sonar_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.range = range;
    packet.angle = angle;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SCANNING_SONAR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SCANNING_SONAR;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SCANNING_SONAR_MIN_LEN, MAVLINK_MSG_ID_SCANNING_SONAR_LEN, MAVLINK_MSG_ID_SCANNING_SONAR_CRC);
}

/**
 * @brief Encode a scanning_sonar struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param scanning_sonar C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_scanning_sonar_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_scanning_sonar_t* scanning_sonar)
{
    return mavlink_msg_scanning_sonar_pack(system_id, component_id, msg, scanning_sonar->time_boot_ms, scanning_sonar->range, scanning_sonar->angle, scanning_sonar->roll, scanning_sonar->pitch, scanning_sonar->yaw);
}

/**
 * @brief Encode a scanning_sonar struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param scanning_sonar C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_scanning_sonar_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_scanning_sonar_t* scanning_sonar)
{
    return mavlink_msg_scanning_sonar_pack_chan(system_id, component_id, chan, msg, scanning_sonar->time_boot_ms, scanning_sonar->range, scanning_sonar->angle, scanning_sonar->roll, scanning_sonar->pitch, scanning_sonar->yaw);
}

/**
 * @brief Send a scanning_sonar message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param range [mm] Measured range
 * @param angle [0.1 degrees] Angle
 * @param roll [0.1 deg] Roll
 * @param pitch [0.1 deg] Pitch.
 * @param yaw [0.1 deg] Heading.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_scanning_sonar_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint16_t range, uint16_t angle, uint16_t roll, uint16_t pitch, uint16_t yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SCANNING_SONAR_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint16_t(buf, 4, range);
    _mav_put_uint16_t(buf, 6, angle);
    _mav_put_uint16_t(buf, 8, roll);
    _mav_put_uint16_t(buf, 10, pitch);
    _mav_put_uint16_t(buf, 12, yaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCANNING_SONAR, buf, MAVLINK_MSG_ID_SCANNING_SONAR_MIN_LEN, MAVLINK_MSG_ID_SCANNING_SONAR_LEN, MAVLINK_MSG_ID_SCANNING_SONAR_CRC);
#else
    mavlink_scanning_sonar_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.range = range;
    packet.angle = angle;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCANNING_SONAR, (const char *)&packet, MAVLINK_MSG_ID_SCANNING_SONAR_MIN_LEN, MAVLINK_MSG_ID_SCANNING_SONAR_LEN, MAVLINK_MSG_ID_SCANNING_SONAR_CRC);
#endif
}

/**
 * @brief Send a scanning_sonar message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_scanning_sonar_send_struct(mavlink_channel_t chan, const mavlink_scanning_sonar_t* scanning_sonar)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_scanning_sonar_send(chan, scanning_sonar->time_boot_ms, scanning_sonar->range, scanning_sonar->angle, scanning_sonar->roll, scanning_sonar->pitch, scanning_sonar->yaw);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCANNING_SONAR, (const char *)scanning_sonar, MAVLINK_MSG_ID_SCANNING_SONAR_MIN_LEN, MAVLINK_MSG_ID_SCANNING_SONAR_LEN, MAVLINK_MSG_ID_SCANNING_SONAR_CRC);
#endif
}

#if MAVLINK_MSG_ID_SCANNING_SONAR_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_scanning_sonar_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, uint16_t range, uint16_t angle, uint16_t roll, uint16_t pitch, uint16_t yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint16_t(buf, 4, range);
    _mav_put_uint16_t(buf, 6, angle);
    _mav_put_uint16_t(buf, 8, roll);
    _mav_put_uint16_t(buf, 10, pitch);
    _mav_put_uint16_t(buf, 12, yaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCANNING_SONAR, buf, MAVLINK_MSG_ID_SCANNING_SONAR_MIN_LEN, MAVLINK_MSG_ID_SCANNING_SONAR_LEN, MAVLINK_MSG_ID_SCANNING_SONAR_CRC);
#else
    mavlink_scanning_sonar_t *packet = (mavlink_scanning_sonar_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->range = range;
    packet->angle = angle;
    packet->roll = roll;
    packet->pitch = pitch;
    packet->yaw = yaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCANNING_SONAR, (const char *)packet, MAVLINK_MSG_ID_SCANNING_SONAR_MIN_LEN, MAVLINK_MSG_ID_SCANNING_SONAR_LEN, MAVLINK_MSG_ID_SCANNING_SONAR_CRC);
#endif
}
#endif

#endif

// MESSAGE SCANNING_SONAR UNPACKING


/**
 * @brief Get field time_boot_ms from scanning_sonar message
 *
 * @return [ms] Timestamp (time since system boot).
 */
static inline uint32_t mavlink_msg_scanning_sonar_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field range from scanning_sonar message
 *
 * @return [mm] Measured range
 */
static inline uint16_t mavlink_msg_scanning_sonar_get_range(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field angle from scanning_sonar message
 *
 * @return [0.1 degrees] Angle
 */
static inline uint16_t mavlink_msg_scanning_sonar_get_angle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Get field roll from scanning_sonar message
 *
 * @return [0.1 deg] Roll
 */
static inline uint16_t mavlink_msg_scanning_sonar_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field pitch from scanning_sonar message
 *
 * @return [0.1 deg] Pitch.
 */
static inline uint16_t mavlink_msg_scanning_sonar_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  10);
}

/**
 * @brief Get field yaw from scanning_sonar message
 *
 * @return [0.1 deg] Heading.
 */
static inline uint16_t mavlink_msg_scanning_sonar_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Decode a scanning_sonar message into a struct
 *
 * @param msg The message to decode
 * @param scanning_sonar C-struct to decode the message contents into
 */
static inline void mavlink_msg_scanning_sonar_decode(const mavlink_message_t* msg, mavlink_scanning_sonar_t* scanning_sonar)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    scanning_sonar->time_boot_ms = mavlink_msg_scanning_sonar_get_time_boot_ms(msg);
    scanning_sonar->range = mavlink_msg_scanning_sonar_get_range(msg);
    scanning_sonar->angle = mavlink_msg_scanning_sonar_get_angle(msg);
    scanning_sonar->roll = mavlink_msg_scanning_sonar_get_roll(msg);
    scanning_sonar->pitch = mavlink_msg_scanning_sonar_get_pitch(msg);
    scanning_sonar->yaw = mavlink_msg_scanning_sonar_get_yaw(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SCANNING_SONAR_LEN? msg->len : MAVLINK_MSG_ID_SCANNING_SONAR_LEN;
        memset(scanning_sonar, 0, MAVLINK_MSG_ID_SCANNING_SONAR_LEN);
    memcpy(scanning_sonar, _MAV_PAYLOAD(msg), len);
#endif
}
