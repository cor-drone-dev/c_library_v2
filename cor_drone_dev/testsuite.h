/** @file
 *    @brief MAVLink comm protocol testsuite generated from cor_drone_dev.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef COR_DRONE_DEV_TESTSUITE_H
#define COR_DRONE_DEV_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_cor_drone_dev(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_common(system_id, component_id, last_msg);
    mavlink_test_cor_drone_dev(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_hover_thrust_estimate(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_hover_thrust_estimate_t packet_in = {
        93372036854775807ULL,93372036854776311ULL,129.0,157.0,185.0,213.0,241.0,269.0,125
    };
    mavlink_hover_thrust_estimate_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.timestamp_sample = packet_in.timestamp_sample;
        packet1.hover_thrust = packet_in.hover_thrust;
        packet1.hover_thrust_var = packet_in.hover_thrust_var;
        packet1.accel_innov = packet_in.accel_innov;
        packet1.accel_innov_var = packet_in.accel_innov_var;
        packet1.accel_innov_test_ratio = packet_in.accel_innov_test_ratio;
        packet1.accel_noise_var = packet_in.accel_noise_var;
        packet1.valid = packet_in.valid;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hover_thrust_estimate_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_hover_thrust_estimate_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hover_thrust_estimate_pack(system_id, component_id, &msg , packet1.timestamp , packet1.timestamp_sample , packet1.hover_thrust , packet1.hover_thrust_var , packet1.accel_innov , packet1.accel_innov_var , packet1.accel_innov_test_ratio , packet1.accel_noise_var , packet1.valid );
    mavlink_msg_hover_thrust_estimate_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hover_thrust_estimate_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.timestamp_sample , packet1.hover_thrust , packet1.hover_thrust_var , packet1.accel_innov , packet1.accel_innov_var , packet1.accel_innov_test_ratio , packet1.accel_noise_var , packet1.valid );
    mavlink_msg_hover_thrust_estimate_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_hover_thrust_estimate_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hover_thrust_estimate_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.timestamp_sample , packet1.hover_thrust , packet1.hover_thrust_var , packet1.accel_innov , packet1.accel_innov_var , packet1.accel_innov_test_ratio , packet1.accel_noise_var , packet1.valid );
    mavlink_msg_hover_thrust_estimate_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("HOVER_THRUST_ESTIMATE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE) != NULL);
#endif
}

static void mavlink_test_cor_drone_dev(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_hover_thrust_estimate(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // COR_DRONE_DEV_TESTSUITE_H
