#pragma once
// MESSAGE HOVER_THRUST_ESTIMATE PACKING

#define MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE 229


typedef struct __mavlink_hover_thrust_estimate_t {
 uint64_t timestamp; /*< [us] Time since system start.*/
 uint64_t timestamp_sample; /*<  Time of corresponding sensor data last used for this estimate.*/
 float hover_thrust; /*<  Estimted hover thrust [0.1 0.9].*/
 float hover_thrust_var; /*<  Estimated hover thrust variance.*/
 float accel_innov; /*<  Innovation of the last acceleration.*/
 float accel_innov_var; /*<  Innovation variance of the last acceleration fusion.*/
 float accel_innov_test_ratio; /*<  Normalized innovation squared test ratio.*/
 float accel_noise_var; /*<  Vertical accleration noise variance form innovation residual.*/
 uint8_t valid; /*<  Boolean indicating whether the hover thrust estimate is valid or not (valid: 1, invalid: 0). Default is 0 (invalid).*/
} mavlink_hover_thrust_estimate_t;

#define MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_LEN 41
#define MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_MIN_LEN 41
#define MAVLINK_MSG_ID_229_LEN 41
#define MAVLINK_MSG_ID_229_MIN_LEN 41

#define MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_CRC 155
#define MAVLINK_MSG_ID_229_CRC 155



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_HOVER_THRUST_ESTIMATE { \
    229, \
    "HOVER_THRUST_ESTIMATE", \
    9, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_hover_thrust_estimate_t, timestamp) }, \
         { "timestamp_sample", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_hover_thrust_estimate_t, timestamp_sample) }, \
         { "hover_thrust", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_hover_thrust_estimate_t, hover_thrust) }, \
         { "hover_thrust_var", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_hover_thrust_estimate_t, hover_thrust_var) }, \
         { "accel_innov", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_hover_thrust_estimate_t, accel_innov) }, \
         { "accel_innov_var", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_hover_thrust_estimate_t, accel_innov_var) }, \
         { "accel_innov_test_ratio", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_hover_thrust_estimate_t, accel_innov_test_ratio) }, \
         { "accel_noise_var", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_hover_thrust_estimate_t, accel_noise_var) }, \
         { "valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_hover_thrust_estimate_t, valid) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_HOVER_THRUST_ESTIMATE { \
    "HOVER_THRUST_ESTIMATE", \
    9, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_hover_thrust_estimate_t, timestamp) }, \
         { "timestamp_sample", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_hover_thrust_estimate_t, timestamp_sample) }, \
         { "hover_thrust", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_hover_thrust_estimate_t, hover_thrust) }, \
         { "hover_thrust_var", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_hover_thrust_estimate_t, hover_thrust_var) }, \
         { "accel_innov", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_hover_thrust_estimate_t, accel_innov) }, \
         { "accel_innov_var", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_hover_thrust_estimate_t, accel_innov_var) }, \
         { "accel_innov_test_ratio", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_hover_thrust_estimate_t, accel_innov_test_ratio) }, \
         { "accel_noise_var", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_hover_thrust_estimate_t, accel_noise_var) }, \
         { "valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_hover_thrust_estimate_t, valid) }, \
         } \
}
#endif

/**
 * @brief Pack a hover_thrust_estimate message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Time since system start.
 * @param timestamp_sample  Time of corresponding sensor data last used for this estimate.
 * @param hover_thrust  Estimted hover thrust [0.1 0.9].
 * @param hover_thrust_var  Estimated hover thrust variance.
 * @param accel_innov  Innovation of the last acceleration.
 * @param accel_innov_var  Innovation variance of the last acceleration fusion.
 * @param accel_innov_test_ratio  Normalized innovation squared test ratio.
 * @param accel_noise_var  Vertical accleration noise variance form innovation residual.
 * @param valid  Boolean indicating whether the hover thrust estimate is valid or not (valid: 1, invalid: 0). Default is 0 (invalid).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hover_thrust_estimate_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint64_t timestamp_sample, float hover_thrust, float hover_thrust_var, float accel_innov, float accel_innov_var, float accel_innov_test_ratio, float accel_noise_var, uint8_t valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, timestamp_sample);
    _mav_put_float(buf, 16, hover_thrust);
    _mav_put_float(buf, 20, hover_thrust_var);
    _mav_put_float(buf, 24, accel_innov);
    _mav_put_float(buf, 28, accel_innov_var);
    _mav_put_float(buf, 32, accel_innov_test_ratio);
    _mav_put_float(buf, 36, accel_noise_var);
    _mav_put_uint8_t(buf, 40, valid);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_LEN);
#else
    mavlink_hover_thrust_estimate_t packet;
    packet.timestamp = timestamp;
    packet.timestamp_sample = timestamp_sample;
    packet.hover_thrust = hover_thrust;
    packet.hover_thrust_var = hover_thrust_var;
    packet.accel_innov = accel_innov;
    packet.accel_innov_var = accel_innov_var;
    packet.accel_innov_test_ratio = accel_innov_test_ratio;
    packet.accel_noise_var = accel_noise_var;
    packet.valid = valid;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_LEN, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_CRC);
}

/**
 * @brief Pack a hover_thrust_estimate message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Time since system start.
 * @param timestamp_sample  Time of corresponding sensor data last used for this estimate.
 * @param hover_thrust  Estimted hover thrust [0.1 0.9].
 * @param hover_thrust_var  Estimated hover thrust variance.
 * @param accel_innov  Innovation of the last acceleration.
 * @param accel_innov_var  Innovation variance of the last acceleration fusion.
 * @param accel_innov_test_ratio  Normalized innovation squared test ratio.
 * @param accel_noise_var  Vertical accleration noise variance form innovation residual.
 * @param valid  Boolean indicating whether the hover thrust estimate is valid or not (valid: 1, invalid: 0). Default is 0 (invalid).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hover_thrust_estimate_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint64_t timestamp_sample,float hover_thrust,float hover_thrust_var,float accel_innov,float accel_innov_var,float accel_innov_test_ratio,float accel_noise_var,uint8_t valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, timestamp_sample);
    _mav_put_float(buf, 16, hover_thrust);
    _mav_put_float(buf, 20, hover_thrust_var);
    _mav_put_float(buf, 24, accel_innov);
    _mav_put_float(buf, 28, accel_innov_var);
    _mav_put_float(buf, 32, accel_innov_test_ratio);
    _mav_put_float(buf, 36, accel_noise_var);
    _mav_put_uint8_t(buf, 40, valid);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_LEN);
#else
    mavlink_hover_thrust_estimate_t packet;
    packet.timestamp = timestamp;
    packet.timestamp_sample = timestamp_sample;
    packet.hover_thrust = hover_thrust;
    packet.hover_thrust_var = hover_thrust_var;
    packet.accel_innov = accel_innov;
    packet.accel_innov_var = accel_innov_var;
    packet.accel_innov_test_ratio = accel_innov_test_ratio;
    packet.accel_noise_var = accel_noise_var;
    packet.valid = valid;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_LEN, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_CRC);
}

/**
 * @brief Encode a hover_thrust_estimate struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param hover_thrust_estimate C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hover_thrust_estimate_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_hover_thrust_estimate_t* hover_thrust_estimate)
{
    return mavlink_msg_hover_thrust_estimate_pack(system_id, component_id, msg, hover_thrust_estimate->timestamp, hover_thrust_estimate->timestamp_sample, hover_thrust_estimate->hover_thrust, hover_thrust_estimate->hover_thrust_var, hover_thrust_estimate->accel_innov, hover_thrust_estimate->accel_innov_var, hover_thrust_estimate->accel_innov_test_ratio, hover_thrust_estimate->accel_noise_var, hover_thrust_estimate->valid);
}

/**
 * @brief Encode a hover_thrust_estimate struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param hover_thrust_estimate C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hover_thrust_estimate_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_hover_thrust_estimate_t* hover_thrust_estimate)
{
    return mavlink_msg_hover_thrust_estimate_pack_chan(system_id, component_id, chan, msg, hover_thrust_estimate->timestamp, hover_thrust_estimate->timestamp_sample, hover_thrust_estimate->hover_thrust, hover_thrust_estimate->hover_thrust_var, hover_thrust_estimate->accel_innov, hover_thrust_estimate->accel_innov_var, hover_thrust_estimate->accel_innov_test_ratio, hover_thrust_estimate->accel_noise_var, hover_thrust_estimate->valid);
}

/**
 * @brief Send a hover_thrust_estimate message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Time since system start.
 * @param timestamp_sample  Time of corresponding sensor data last used for this estimate.
 * @param hover_thrust  Estimted hover thrust [0.1 0.9].
 * @param hover_thrust_var  Estimated hover thrust variance.
 * @param accel_innov  Innovation of the last acceleration.
 * @param accel_innov_var  Innovation variance of the last acceleration fusion.
 * @param accel_innov_test_ratio  Normalized innovation squared test ratio.
 * @param accel_noise_var  Vertical accleration noise variance form innovation residual.
 * @param valid  Boolean indicating whether the hover thrust estimate is valid or not (valid: 1, invalid: 0). Default is 0 (invalid).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_hover_thrust_estimate_send(mavlink_channel_t chan, uint64_t timestamp, uint64_t timestamp_sample, float hover_thrust, float hover_thrust_var, float accel_innov, float accel_innov_var, float accel_innov_test_ratio, float accel_noise_var, uint8_t valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, timestamp_sample);
    _mav_put_float(buf, 16, hover_thrust);
    _mav_put_float(buf, 20, hover_thrust_var);
    _mav_put_float(buf, 24, accel_innov);
    _mav_put_float(buf, 28, accel_innov_var);
    _mav_put_float(buf, 32, accel_innov_test_ratio);
    _mav_put_float(buf, 36, accel_noise_var);
    _mav_put_uint8_t(buf, 40, valid);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE, buf, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_LEN, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_CRC);
#else
    mavlink_hover_thrust_estimate_t packet;
    packet.timestamp = timestamp;
    packet.timestamp_sample = timestamp_sample;
    packet.hover_thrust = hover_thrust;
    packet.hover_thrust_var = hover_thrust_var;
    packet.accel_innov = accel_innov;
    packet.accel_innov_var = accel_innov_var;
    packet.accel_innov_test_ratio = accel_innov_test_ratio;
    packet.accel_noise_var = accel_noise_var;
    packet.valid = valid;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE, (const char *)&packet, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_LEN, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_CRC);
#endif
}

/**
 * @brief Send a hover_thrust_estimate message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_hover_thrust_estimate_send_struct(mavlink_channel_t chan, const mavlink_hover_thrust_estimate_t* hover_thrust_estimate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_hover_thrust_estimate_send(chan, hover_thrust_estimate->timestamp, hover_thrust_estimate->timestamp_sample, hover_thrust_estimate->hover_thrust, hover_thrust_estimate->hover_thrust_var, hover_thrust_estimate->accel_innov, hover_thrust_estimate->accel_innov_var, hover_thrust_estimate->accel_innov_test_ratio, hover_thrust_estimate->accel_noise_var, hover_thrust_estimate->valid);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE, (const char *)hover_thrust_estimate, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_LEN, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_CRC);
#endif
}

#if MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_hover_thrust_estimate_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint64_t timestamp_sample, float hover_thrust, float hover_thrust_var, float accel_innov, float accel_innov_var, float accel_innov_test_ratio, float accel_noise_var, uint8_t valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, timestamp_sample);
    _mav_put_float(buf, 16, hover_thrust);
    _mav_put_float(buf, 20, hover_thrust_var);
    _mav_put_float(buf, 24, accel_innov);
    _mav_put_float(buf, 28, accel_innov_var);
    _mav_put_float(buf, 32, accel_innov_test_ratio);
    _mav_put_float(buf, 36, accel_noise_var);
    _mav_put_uint8_t(buf, 40, valid);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE, buf, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_LEN, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_CRC);
#else
    mavlink_hover_thrust_estimate_t *packet = (mavlink_hover_thrust_estimate_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->timestamp_sample = timestamp_sample;
    packet->hover_thrust = hover_thrust;
    packet->hover_thrust_var = hover_thrust_var;
    packet->accel_innov = accel_innov;
    packet->accel_innov_var = accel_innov_var;
    packet->accel_innov_test_ratio = accel_innov_test_ratio;
    packet->accel_noise_var = accel_noise_var;
    packet->valid = valid;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE, (const char *)packet, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_LEN, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_CRC);
#endif
}
#endif

#endif

// MESSAGE HOVER_THRUST_ESTIMATE UNPACKING


/**
 * @brief Get field timestamp from hover_thrust_estimate message
 *
 * @return [us] Time since system start.
 */
static inline uint64_t mavlink_msg_hover_thrust_estimate_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field timestamp_sample from hover_thrust_estimate message
 *
 * @return  Time of corresponding sensor data last used for this estimate.
 */
static inline uint64_t mavlink_msg_hover_thrust_estimate_get_timestamp_sample(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Get field hover_thrust from hover_thrust_estimate message
 *
 * @return  Estimted hover thrust [0.1 0.9].
 */
static inline float mavlink_msg_hover_thrust_estimate_get_hover_thrust(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field hover_thrust_var from hover_thrust_estimate message
 *
 * @return  Estimated hover thrust variance.
 */
static inline float mavlink_msg_hover_thrust_estimate_get_hover_thrust_var(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field accel_innov from hover_thrust_estimate message
 *
 * @return  Innovation of the last acceleration.
 */
static inline float mavlink_msg_hover_thrust_estimate_get_accel_innov(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field accel_innov_var from hover_thrust_estimate message
 *
 * @return  Innovation variance of the last acceleration fusion.
 */
static inline float mavlink_msg_hover_thrust_estimate_get_accel_innov_var(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field accel_innov_test_ratio from hover_thrust_estimate message
 *
 * @return  Normalized innovation squared test ratio.
 */
static inline float mavlink_msg_hover_thrust_estimate_get_accel_innov_test_ratio(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field accel_noise_var from hover_thrust_estimate message
 *
 * @return  Vertical accleration noise variance form innovation residual.
 */
static inline float mavlink_msg_hover_thrust_estimate_get_accel_noise_var(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field valid from hover_thrust_estimate message
 *
 * @return  Boolean indicating whether the hover thrust estimate is valid or not (valid: 1, invalid: 0). Default is 0 (invalid).
 */
static inline uint8_t mavlink_msg_hover_thrust_estimate_get_valid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  40);
}

/**
 * @brief Decode a hover_thrust_estimate message into a struct
 *
 * @param msg The message to decode
 * @param hover_thrust_estimate C-struct to decode the message contents into
 */
static inline void mavlink_msg_hover_thrust_estimate_decode(const mavlink_message_t* msg, mavlink_hover_thrust_estimate_t* hover_thrust_estimate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    hover_thrust_estimate->timestamp = mavlink_msg_hover_thrust_estimate_get_timestamp(msg);
    hover_thrust_estimate->timestamp_sample = mavlink_msg_hover_thrust_estimate_get_timestamp_sample(msg);
    hover_thrust_estimate->hover_thrust = mavlink_msg_hover_thrust_estimate_get_hover_thrust(msg);
    hover_thrust_estimate->hover_thrust_var = mavlink_msg_hover_thrust_estimate_get_hover_thrust_var(msg);
    hover_thrust_estimate->accel_innov = mavlink_msg_hover_thrust_estimate_get_accel_innov(msg);
    hover_thrust_estimate->accel_innov_var = mavlink_msg_hover_thrust_estimate_get_accel_innov_var(msg);
    hover_thrust_estimate->accel_innov_test_ratio = mavlink_msg_hover_thrust_estimate_get_accel_innov_test_ratio(msg);
    hover_thrust_estimate->accel_noise_var = mavlink_msg_hover_thrust_estimate_get_accel_noise_var(msg);
    hover_thrust_estimate->valid = mavlink_msg_hover_thrust_estimate_get_valid(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_LEN? msg->len : MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_LEN;
        memset(hover_thrust_estimate, 0, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_LEN);
    memcpy(hover_thrust_estimate, _MAV_PAYLOAD(msg), len);
#endif
}
