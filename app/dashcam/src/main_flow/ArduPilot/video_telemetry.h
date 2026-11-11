/*
  telemetry carried in the H.264 video, in the AP_CameraGimbal format:
  an unregistered user data SEI holding apcg.telemetry.v1 JSON, put
  before the first slice of each frame
 */
#pragma once

#include <stdint.h>
#include <sys/time.h>

/*
  called from the MAVLink code as the flight controller's messages
  arrive; the message structures are declared, not included, so the
  video code doesn't need the MAVLink headers
 */
struct __mavlink_attitude_t;
struct __mavlink_global_position_int_t;
struct __mavlink_system_time_t;
void video_telemetry_attitude(const struct __mavlink_attitude_t *m);
void video_telemetry_position(const struct __mavlink_global_position_int_t *m);
void video_telemetry_system_time(const struct __mavlink_system_time_t *m);

/*
  a copy of an Annex-B H.264 frame with a telemetry SEI before its first
  slice, in a buffer kept for the next call, with room for spare_bytes
  after the frame; NULL if the frame has no slice or there's no memory,
  so the frame should go as it is
 */
uint8_t *video_telemetry_insert(const uint8_t *frame, unsigned length,
                                const struct timeval *tv, unsigned spare_bytes,
                                unsigned *new_length);
