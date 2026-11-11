/*
  telemetry carried in the H.264 video, in the format AP_CameraGimbal
  uses so that its tools and MAVProxy's mavvidplay can read it: each
  frame gets an unregistered user data SEI (payload type 5) with the
  UUID 8d646b4e-556f-4a90-8b7c-35e629510321 followed by
  apcg.telemetry.v1 JSON, put immediately before the frame's first
  slice.

  The SkyViper's camera is fixed to the vehicle, looking straight
  ahead, so the "gimbal" attitude is the vehicle's roll and pitch with
  zero relative yaw.

  The SDK's printf has no floating point, so the JSON is written from
  fixed point integers.
 */
#include <FreeRTOS.h>
#include <task.h>
#include <nonstdlib.h>
#include <string.h>
#include <stdbool.h>
#include <mavlink_core.h>
#include "dev_console.h"
#include "video_telemetry.h"

#define TELEMETRY_MAX_AGE_MS 10000  // older values are given as null

/*
  the camera's horizontal field of view, edge to edge: at 140mm from a
  ruler the 1280x720 preview showed 142mm left and 143mm right of its
  centre.  The lens has strong barrel distortion, so near the centre
  of the picture it's more like a 74 degree pinhole camera.
 */
#define CAMERA_HFOV "91.0"
#define JSON_MAX 640   // all the fields are about 500 bytes
#define SEI_MAX (JSON_MAX + 32)

static const uint8_t apcg_uuid[16] = {
    0x8d, 0x64, 0x6b, 0x4e, 0x55, 0x6f, 0x4a, 0x90,
    0x8b, 0x7c, 0x35, 0xe6, 0x29, 0x51, 0x03, 0x21,
};

/*
  the latest values from the flight controller, in the units they're
  written in; changed by the MAVLink task and read by the video task
  inside critical sections
 */
static struct {
    bool have_attitude, have_position, have_time;
    uint32_t attitude_ms, position_ms, time_ms;  // when received
    int32_t roll_urad, pitch_urad, yaw_urad;
    int32_t lat_e7, lon_e7;
    int32_t alt_mm, relative_alt_mm;
    int16_t vn_cm_s, ve_cm_s, vd_cm_s;
    int32_t heading_urad;                        // -1 if unknown
    uint64_t unix_us;
    // the time the web interface or phone app gave, for when the
    // flight controller doesn't know it
    bool have_given_time;
    uint32_t given_ms;
    uint64_t given_unix_us;
} telem;

/*
  a given time is only to the second, and the web interface gives it
  every 20 seconds: keep the first unless it's out by more than this,
  so the time in a recording doesn't jump about
 */
#define GIVEN_TIME_SLACK_US 2000000ULL

void video_telemetry_attitude(const mavlink_attitude_t *m)
{
    int32_t roll = (int32_t)(m->roll * 1.0e6f);
    int32_t pitch = (int32_t)(m->pitch * 1.0e6f);
    int32_t yaw = (int32_t)(m->yaw * 1.0e6f);
    taskENTER_CRITICAL();
    telem.roll_urad = roll;
    telem.pitch_urad = pitch;
    telem.yaw_urad = yaw;
    telem.attitude_ms = get_time_boot_ms();
    telem.have_attitude = true;
    taskEXIT_CRITICAL();
}

void video_telemetry_position(const mavlink_global_position_int_t *m)
{
    // hdg is in centidegrees, UINT16_MAX if unknown
    int32_t heading = m->hdg == UINT16_MAX ? -1 :
        (int32_t)(((int64_t)m->hdg * 1745329 + 5000) / 10000);
    taskENTER_CRITICAL();
    telem.lat_e7 = m->lat;
    telem.lon_e7 = m->lon;
    telem.alt_mm = m->alt;
    telem.relative_alt_mm = m->relative_alt;
    telem.vn_cm_s = m->vx;
    telem.ve_cm_s = m->vy;
    telem.vd_cm_s = m->vz;
    telem.heading_urad = heading;
    telem.position_ms = get_time_boot_ms();
    telem.have_position = true;
    taskEXIT_CRITICAL();
}

void video_telemetry_system_time(const mavlink_system_time_t *m)
{
    if (m->time_unix_usec == 0) {
        // the flight controller doesn't know the time
        return;
    }
    taskENTER_CRITICAL();
    telem.unix_us = m->time_unix_usec;
    telem.time_ms = get_time_boot_ms();
    telem.have_time = true;
    taskEXIT_CRITICAL();
}

void video_telemetry_given_time(uint32_t unix_seconds)
{
    const uint64_t given_us = (uint64_t)unix_seconds * 1000000;
    taskENTER_CRITICAL();
    const uint32_t now_ms = get_time_boot_ms();
    if (telem.have_given_time) {
        const uint64_t ours = telem.given_unix_us + (uint64_t)(now_ms - telem.given_ms) * 1000;
        const uint64_t diff = ours > given_us ? ours - given_us : given_us - ours;
        if (diff <= GIVEN_TIME_SLACK_US) {
            taskEXIT_CRITICAL();
            return;
        }
    }
    telem.given_unix_us = given_us;
    telem.given_ms = now_ms;
    telem.have_given_time = true;
    taskEXIT_CRITICAL();
}

/*
  a JSON string being built, which stops growing when full
 */
struct json {
    char *buf;
    unsigned len, size;
    bool full;
};

static void put_str(struct json *j, const char *s)
{
    unsigned n = strlen(s);
    if (j->full || j->len + n > j->size) {
        j->full = true;
        return;
    }
    memcpy(j->buf + j->len, s, n);
    j->len += n;
}

static void put_uint(struct json *j, uint64_t v, unsigned min_digits)
{
    char digits[21];
    unsigned i = sizeof(digits);
    digits[--i] = 0;
    do {
        digits[--i] = '0' + (char)(v % 10);
        v /= 10;
        if (min_digits > 0) {
            min_digits--;
        }
    } while (v != 0 || min_digits > 0);
    put_str(j, &digits[i]);
}

static void put_int(struct json *j, int64_t v)
{
    if (v < 0) {
        put_str(j, "-");
        put_uint(j, -(uint64_t)v, 0);
    } else {
        put_uint(j, v, 0);
    }
}

// v / 10^decimals, with that many decimal places
static void put_fixed(struct json *j, int64_t v, unsigned decimals)
{
    uint64_t scale = 1;
    unsigned i;
    for (i = 0; i < decimals; i++) {
        scale *= 10;
    }
    uint64_t a = v < 0 ? -(uint64_t)v : (uint64_t)v;
    if (v < 0) {
        put_str(j, "-");
    }
    put_uint(j, a / scale, 0);
    put_str(j, ".");
    put_uint(j, a % scale, decimals);
}

static void put_age(struct json *j, uint32_t age_ms)
{
    put_str(j, ",\"age_ms\":");
    put_uint(j, age_ms, 0);
    put_str(j, "}");
}

static bool fresh(bool received, uint32_t received_ms, uint32_t now_ms)
{
    return received && now_ms - received_ms <= TELEMETRY_MAX_AGE_MS;
}

/*
  the apcg.telemetry.v1 JSON for a frame; the length, or 0 if it
  doesn't fit
 */
static unsigned telemetry_json(uint32_t pts90k, char *buf, unsigned size)
{
    struct json j = { buf, 0, size, false };
    uint32_t now_ms;
    typeof(telem) t;

    // the time is taken with the copy, so nothing in it is newer
    taskENTER_CRITICAL();
    t = telem;
    now_ms = get_time_boot_ms();
    taskEXIT_CRITICAL();

    put_str(&j, "{\"schema\":\"apcg.telemetry.v1\",\"pts90k\":");
    put_uint(&j, pts90k, 0);
    put_str(&j, ",\"utc_us\":");
    if (t.have_time) {
        // the flight controller's last time, carried on by our clock
        put_uint(&j, t.unix_us + (uint64_t)(now_ms - t.time_ms) * 1000, 0);
    } else if (t.have_given_time) {
        // the time we were given, e.g. with no GPS: good to a second or so
        put_uint(&j, t.given_unix_us + (uint64_t)(now_ms - t.given_ms) * 1000, 0);
    } else {
        put_str(&j, "null");
    }

    bool have_position = fresh(t.have_position, t.position_ms, now_ms);
    uint32_t position_age = now_ms - t.position_ms;
    put_str(&j, ",\"position\":");
    // without a position the flight controller sends 0 latitude and longitude
    if (have_position && (t.lat_e7 != 0 || t.lon_e7 != 0)) {
        put_str(&j, "{\"lat_e7\":");
        put_int(&j, t.lat_e7);
        put_str(&j, ",\"lon_e7\":");
        put_int(&j, t.lon_e7);
        put_str(&j, ",\"alt_amsl_m\":");
        put_fixed(&j, t.alt_mm, 3);
        put_str(&j, ",\"alt_relative_m\":");
        put_fixed(&j, t.relative_alt_mm, 3);
        put_age(&j, position_age);
    } else {
        put_str(&j, "null");
    }

    put_str(&j, ",\"velocity\":");
    if (have_position) {
        put_str(&j, "{\"vn_m_s\":");
        put_fixed(&j, t.vn_cm_s, 2);
        put_str(&j, ",\"ve_m_s\":");
        put_fixed(&j, t.ve_cm_s, 2);
        put_str(&j, ",\"vd_m_s\":");
        put_fixed(&j, t.vd_cm_s, 2);
        put_age(&j, position_age);
    } else {
        put_str(&j, "null");
    }

    bool have_attitude = fresh(t.have_attitude, t.attitude_ms, now_ms);
    uint32_t attitude_age = now_ms - t.attitude_ms;
    put_str(&j, ",\"vehicle_attitude\":");
    if (have_attitude) {
        put_str(&j, "{\"roll_rad\":");
        put_fixed(&j, t.roll_urad, 6);
        put_str(&j, ",\"pitch_rad\":");
        put_fixed(&j, t.pitch_urad, 6);
        put_str(&j, ",\"yaw_rad\":");
        put_fixed(&j, t.yaw_urad, 6);
        put_age(&j, attitude_age);
    } else {
        put_str(&j, "null");
    }

    // the camera is fixed: level-referenced roll and pitch are the
    // vehicle's, and its yaw relative to the vehicle is zero
    put_str(&j, ",\"gimbal_attitude\":");
    if (have_attitude) {
        put_str(&j, "{\"roll_rad\":");
        put_fixed(&j, t.roll_urad, 6);
        put_str(&j, ",\"pitch_rad\":");
        put_fixed(&j, t.pitch_urad, 6);
        put_str(&j, ",\"yaw_rad\":0.000000");
        put_age(&j, attitude_age);
    } else {
        put_str(&j, "null");
    }

    put_str(&j, ",\"heading_rad\":");
    if (have_position && t.heading_urad >= 0) {
        put_fixed(&j, t.heading_urad, 6);
    } else {
        put_str(&j, "null");
    }
    put_str(&j, ",\"zoom\":1.000,\"hfov_deg\":" CAMERA_HFOV "}");

    return j.full ? 0 : j.len;
}

/*
  the SEI NAL unit, with its start code, for some JSON; its length, or
  0 if it doesn't fit
 */
static unsigned telemetry_sei(const char *json, unsigned json_len, uint8_t *rbsp,
                              uint8_t *out, unsigned size)
{
    unsigned used = 0, written = 0, zeros = 0, i;
    unsigned payload = sizeof(apcg_uuid) + json_len;

    if (json_len == 0 || json_len > JSON_MAX) {
        return 0;
    }
    rbsp[used++] = 5;  // user_data_unregistered
    while (payload >= 255) {
        rbsp[used++] = 255;
        payload -= 255;
    }
    rbsp[used++] = (uint8_t)payload;
    memcpy(&rbsp[used], apcg_uuid, sizeof(apcg_uuid));
    used += sizeof(apcg_uuid);
    memcpy(&rbsp[used], json, json_len);
    used += json_len;
    rbsp[used++] = 0x80;  // rbsp_trailing_bits

    if (size < 5) {
        return 0;
    }
    out[written++] = 0;
    out[written++] = 0;
    out[written++] = 0;
    out[written++] = 1;
    out[written++] = 6;  // nal_unit_type SEI
    for (i = 0; i < used; i++) {
        // emulation prevention
        if (zeros == 2 && rbsp[i] <= 3) {
            if (written == size) {
                return 0;
            }
            out[written++] = 3;
            zeros = 0;
        }
        if (written == size) {
            return 0;
        }
        out[written++] = rbsp[i];
        zeros = rbsp[i] == 0 ? zeros + 1 : 0;
    }
    return written;
}

/*
  where the start code of the frame's first slice (NAL unit types 1 to
  5) begins, or -1 if it has none
 */
static int first_slice(const uint8_t *frame, unsigned length)
{
    unsigned i;
    for (i = 0; i + 3 < length; i++) {
        if (frame[i] == 0 && frame[i + 1] == 0 && frame[i + 2] == 1) {
            uint8_t type = frame[i + 3] & 0x1f;
            if (type >= 1 && type <= 5) {
                return (i > 0 && frame[i - 1] == 0) ? (int)i - 1 : (int)i;
            }
            i += 2;
        }
    }
    return -1;
}

/*
  each stream is sent by its own task, so has its own buffers, which
  are static to keep them off those tasks' stacks
 */
static struct telemetry_buffers {
    uint8_t *frame;          // the frame with its SEI
    unsigned frame_size;
    char json[JSON_MAX];
    uint8_t rbsp[SEI_MAX];
    uint8_t sei[SEI_MAX];
} buffers[VIDEO_TELEMETRY_STREAMS];

uint8_t *video_telemetry_insert(enum video_telemetry_stream stream,
                                const uint8_t *frame, unsigned length,
                                const struct timeval *tv, bool sei_last,
                                unsigned spare_bytes, unsigned *new_length)
{
    struct telemetry_buffers *b = &buffers[stream];

    int slice = first_slice(frame, length);
    if (slice < 0) {
        return NULL;
    }
    unsigned at = sei_last ? length : (unsigned)slice;

    // the frame's capture time in 90kHz ticks
    uint32_t pts90k = (uint32_t)((uint64_t)tv->tv_sec * 90000 + (uint64_t)tv->tv_usec * 9 / 100);
    unsigned json_len = telemetry_json(pts90k, b->json, sizeof(b->json));
    unsigned sei_len = telemetry_sei(b->json, json_len, b->rbsp, b->sei, sizeof(b->sei));
    if (sei_len == 0) {
        return NULL;
    }

    unsigned needed = length + sei_len + spare_bytes;
    if (needed > b->frame_size) {
        // grow by a quarter more, so a few bigger I-frames don't each
        // need another allocation
        unsigned size = needed + needed / 4;
        if (b->frame != NULL) {
            vPortFree(b->frame);
        }
        b->frame = pvPortMalloc(size, GFP_KERNEL, MODULE_APP);
        b->frame_size = b->frame != NULL ? size : 0;
        if (b->frame == NULL) {
            return NULL;
        }
    }

    memcpy(b->frame, frame, at);
    memcpy(b->frame + at, b->sei, sei_len);
    memcpy(b->frame + at + sei_len, frame + at, length - at);
    *new_length = length + sei_len;
    return b->frame;
}
