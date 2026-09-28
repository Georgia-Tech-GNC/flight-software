#ifndef ZED_F9P_H
#define ZED_F9P_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define ZED_F9P_MAX_PAYLOAD 128u
#define ZED_F9P_RTCM_MAX_FRAME 1029u // 3 header + 1023 payload + 3 crc

// protocol bits
#define ZED_F9P_PROT_UBX 0x01u
#define ZED_F9P_PROT_NMEA 0x02u
#define ZED_F9P_PROT_RTCM3 0x04u

// cfg keys
#define ZED_F9P_KEY_INPROT(port, prot) (0x10710000u + ((uint32_t)(port) << 17) + (uint32_t)(prot))
#define ZED_F9P_KEY_OUTPROT(port, prot) (0x10720000u + ((uint32_t)(port) << 17) + (uint32_t)(prot))
#define ZED_F9P_KEY_MSGOUT(msg, port) ((uint32_t)(msg) + (uint32_t)(port))
#define ZED_F9P_KEY_UART1_BAUDRATE 0x40520001u
#define ZED_F9P_KEY_UART2_BAUDRATE 0x40530001u
#define ZED_F9P_KEY_RATE_MEAS 0x30210001u // ms
#define ZED_F9P_KEY_RATE_NAV 0x30210002u
#define ZED_F9P_KEY_NAVSPG_DYNMODEL 0x20110021u
#define ZED_F9P_KEY_NAVHPG_DGNSSMODE 0x20140011u // 2 float, 3 fixed
#define ZED_F9P_KEY_TMODE_MODE 0x20030001u // 0 off, 1 survey in, 2 fixed
#define ZED_F9P_KEY_TMODE_POS_TYPE 0x20030002u // 0 ecef, 1 llh
#define ZED_F9P_KEY_TMODE_LAT 0x40030009u // 1e-7 deg
#define ZED_F9P_KEY_TMODE_LON 0x4003000au
#define ZED_F9P_KEY_TMODE_HEIGHT 0x4003000bu // cm
#define ZED_F9P_KEY_TMODE_LAT_HP 0x2003000cu // 1e-9 deg
#define ZED_F9P_KEY_TMODE_LON_HP 0x2003000du
#define ZED_F9P_KEY_TMODE_HEIGHT_HP 0x2003000eu // 0.1 mm
#define ZED_F9P_KEY_TMODE_FIXED_POS_ACC 0x4003000fu // 0.1 mm
#define ZED_F9P_KEY_TMODE_SVIN_MIN_DUR 0x40030010u // s
#define ZED_F9P_KEY_TMODE_SVIN_ACC_LIMIT 0x40030011u // 0.1 mm

// msgout keys i2c
#define ZED_F9P_MSGOUT_NAV_PVT 0x20910006u
#define ZED_F9P_MSGOUT_NAV_HPPOSLLH 0x20910033u
#define ZED_F9P_MSGOUT_NAV_RELPOSNED 0x2091008du
#define ZED_F9P_MSGOUT_NAV_SVIN 0x20910088u
#define ZED_F9P_MSGOUT_RTCM_1005 0x209102bdu
#define ZED_F9P_MSGOUT_RTCM_1074 0x2091035eu
#define ZED_F9P_MSGOUT_RTCM_1084 0x20910363u
#define ZED_F9P_MSGOUT_RTCM_1094 0x20910368u
#define ZED_F9P_MSGOUT_RTCM_1124 0x2091036du
#define ZED_F9P_MSGOUT_RTCM_1230 0x20910303u

// pvt valid bits
#define ZED_F9P_VALID_DATE 0x01u
#define ZED_F9P_VALID_TIME 0x02u
#define ZED_F9P_VALID_FULLY_RESOLVED 0x04u

typedef enum {
    ZED_F9P_FIX_NONE = 0,
    ZED_F9P_FIX_DEAD_RECKONING,
    ZED_F9P_FIX_2D,
    ZED_F9P_FIX_3D,
    ZED_F9P_FIX_GNSS_DR,
    ZED_F9P_FIX_TIME_ONLY
} zed_f9p_fix_t;

typedef enum {
    ZED_F9P_CARRIER_NONE = 0, // standard gnss
    ZED_F9P_CARRIER_FLOAT, // rtk float
    ZED_F9P_CARRIER_FIXED // rtk fixed
} zed_f9p_carrier_t;

typedef enum {
    ZED_F9P_DYN_PORTABLE = 0,
    ZED_F9P_DYN_STATIONARY = 2,
    ZED_F9P_DYN_PEDESTRIAN,
    ZED_F9P_DYN_AUTOMOTIVE,
    ZED_F9P_DYN_SEA,
    ZED_F9P_DYN_AIRBORNE_1G,
    ZED_F9P_DYN_AIRBORNE_2G,
    ZED_F9P_DYN_AIRBORNE_4G
} zed_f9p_dyn_model_t;

typedef struct {
    uint16_t year;
    uint8_t month, day, hour, minute, second;
    uint8_t valid, flags, flags2;
    uint16_t flags3;
    uint32_t time_acc_ns;
    int32_t nano;
    int32_t ground_speed_mms, heading_motion_1e5_deg, heading_vehicle_1e5_deg;
    uint32_t speed_acc_mms, heading_acc_1e5_deg;
    uint16_t pdop_1e2;
    int16_t magnetic_declination_1e2_deg;
    uint16_t magnetic_acc_1e2_deg;
    uint32_t itow_ms; //GPS time of week ms
    int32_t lon_1e7_deg; //longitude 10 millionths degree
    int32_t lat_1e7_deg; //latitude 10 millionths degree
    int32_t height_mm; //height above ellipsoid
    int32_t hmsl_mm; // height above mean sea level
    uint32_t h_acc_mm; //horizontal accuracy estimate
    uint32_t v_acc_mm; //vertical accuracy estimate
    int32_t  vel_n_mms; //north
    int32_t  vel_e_mms; //east
    int32_t  vel_d_mms; //down
    uint8_t fix_type; //zed_f9p_fix_t
    uint8_t num_sats; //satellites used
    bool fix_ok; //receiver considers fix usable
    bool corrections_applied; //differential corrections used
    uint8_t carrier_soln; //zed_f9p_carrier_t
    uint8_t correction_age; //0 none, 1 <1s, 2 1-2s, 3 2-5s, 4 5-10s ... 12 >=120s
    bool llh_valid; //position fields valid
} zed_f9p_pvt_t;

typedef struct {
    uint32_t itow_ms;
    int64_t lon_1e9_deg;
    int64_t lat_1e9_deg;
    int64_t height_1e1_mm; //above ellipsoid, 0.1 mm
    int64_t hmsl_1e1_mm;
    uint32_t h_acc_1e1_mm;
    uint32_t v_acc_1e1_mm;
    bool llh_valid;
} zed_f9p_hpposllh_t;

typedef struct {
    uint32_t itow_ms;
    uint16_t ref_station_id;
    int64_t rel_n_1e1_mm, rel_e_1e1_mm, rel_d_1e1_mm;
    int64_t length_1e1_mm;
    int32_t heading_1e5_deg;
    uint32_t acc_n_1e1_mm, acc_e_1e1_mm, acc_d_1e1_mm, acc_length_1e1_mm;
    uint32_t acc_heading_1e5_deg;
    uint32_t flags;
    bool fix_ok;
    bool corrections_applied;
    bool rel_pos_valid;
    bool heading_valid;
    uint8_t carrier_soln;
} zed_f9p_relposned_t;

typedef struct {
    uint32_t itow_ms;
    uint32_t dur_s;
    int64_t mean_x_1e1_mm, mean_y_1e1_mm, mean_z_1e1_mm; //ecef
    uint32_t mean_acc_1e1_mm;
    uint32_t obs;
    bool valid;
    bool active;
} zed_f9p_svin_t;

typedef enum {
    ZED_F9P_WAIT_SYNC1 = 0,
    ZED_F9P_WAIT_SYNC2,
    ZED_F9P_WAIT_CLASS,
    ZED_F9P_WAIT_ID,
    ZED_F9P_WAIT_LEN_LO,
    ZED_F9P_WAIT_LEN_HI,
    ZED_F9P_WAIT_PAYLOAD,
    ZED_F9P_WAIT_CK_A,
    ZED_F9P_WAIT_CK_B
} zed_f9p_state_t;

typedef enum {
    ZED_F9P_COMMAND_IDLE = 0, ZED_F9P_COMMAND_WAITING,
    ZED_F9P_COMMAND_ACK, ZED_F9P_COMMAND_NAK,
    ZED_F9P_COMMAND_TIMEOUT, ZED_F9P_COMMAND_IO_ERROR
} zed_f9p_command_status_t;

typedef enum {
    ZED_F9P_PORT_I2C = 0, ZED_F9P_PORT_UART1, ZED_F9P_PORT_UART2,
    ZED_F9P_PORT_USB, ZED_F9P_PORT_SPI
} zed_f9p_port_t;

typedef bool (*zed_f9p_write_fn)(void *context, const uint8_t *bytes, size_t length);

typedef struct {
    uint32_t key;
    uint64_t value; //raw bits
} zed_f9p_cfg_t;

typedef struct {
    uint32_t pvt_sequence;
    uint32_t malformed_messages;
    zed_f9p_command_status_t command_status;
    uint8_t command_class, command_id;
    uint32_t command_started_ms, command_timeout_ms;
    zed_f9p_state_t state;
    uint8_t  msg_class;
    uint8_t  msg_id;
    uint16_t payload_len;
    uint16_t payload_pos;
    uint8_t  payload[ZED_F9P_MAX_PAYLOAD];
    uint8_t  ck_a; //checksum accumulated
    uint8_t  ck_b;
    uint8_t  rx_ck_a;//First checksum byte

    uint32_t frames_ok; //passed checksum
    uint32_t checksum_errors; //failed checksum
    uint32_t length_rejects; //length is invalid

    zed_f9p_pvt_t pvt;
    bool pvt_valid;//one solution decoded
    uint32_t pvt_seen_sequence, pvt_rx_ms;

    zed_f9p_hpposllh_t hpposllh;
    zed_f9p_relposned_t relposned;
    zed_f9p_svin_t svin;
    bool hpposllh_valid, relposned_valid, svin_valid;
    uint32_t hpposllh_sequence, relposned_sequence, svin_sequence;
} zed_f9p_t;


typedef struct {
    uint16_t length; //payload length
    uint16_t pos;
    uint32_t frames_ok;
    uint32_t crc_errors;
    uint8_t frame[ZED_F9P_RTCM_MAX_FRAME];
} zed_f9p_rtcm_t;

/** Computes two-byte checksum for UBX message
 * Uses 8-bit Fletcher algorithm. Checksum covers message class, ID, length field, and payload
 * Excludes the two sync characters at the start and the checksum bytes themselves
 *
 * @param bytes   pointer to the first byte to include, which is the message class
 * @param length  number of bytes to include, equal to the payload length plus four
 * @param ck_a    set to the first checksum byte
 * @param ck_b    set to the second checksum byte
 */
void zed_f9p_checksum(const uint8_t *bytes, size_t length, uint8_t *ck_a, uint8_t *ck_b);

//Reset driver to starting state
void zed_f9p_init(zed_f9p_t *dev);

/** Feeds one received byte into parser
 * @returns true when this byte completed a message that passed its checksum
 */
bool zed_f9p_feed(zed_f9p_t *dev, uint8_t byte);

size_t zed_f9p_feed_buffer(zed_f9p_t *dev, const uint8_t *bytes, size_t length);

void zed_f9p_reset_parser(zed_f9p_t *dev);

/** Copies out most recent navigation solution
 * @returns false if no solution has been decoded yet
 */
bool zed_f9p_get_pvt(const zed_f9p_t *dev, zed_f9p_pvt_t *out);
bool zed_f9p_get_hpposllh(const zed_f9p_t *dev, zed_f9p_hpposllh_t *out);
bool zed_f9p_get_relposned(const zed_f9p_t *dev, zed_f9p_relposned_t *out);
bool zed_f9p_get_svin(const zed_f9p_t *dev, zed_f9p_svin_t *out);

uint32_t zed_f9p_pvt_age_ms(const zed_f9p_t *dev, uint32_t now_ms);

bool zed_f9p_pvt_has_position(const zed_f9p_pvt_t *pvt);
bool zed_f9p_pvt_has_time(const zed_f9p_pvt_t *pvt);

size_t zed_f9p_encode(uint8_t cls, uint8_t id, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);

/** Sends one cfg-valset (ram layer) and waits on ack/nak through feed + tick
 * one command at a time, a late ack after a timeout can land on the next one
 * @returns false on bad args, busy, or write failure
 */
bool zed_f9p_configure(zed_f9p_t *dev, const zed_f9p_cfg_t *items, size_t count, zed_f9p_write_fn write_fn, void *context, uint32_t now_ms, uint32_t timeout_ms);
zed_f9p_command_status_t zed_f9p_command_status(const zed_f9p_t *dev);
void zed_f9p_tick(zed_f9p_t *dev, uint32_t now_ms);

bool zed_f9p_start_pvt(zed_f9p_t *dev, zed_f9p_port_t port, uint16_t period_ms, zed_f9p_write_fn write_fn, void *context, uint32_t now_ms, uint32_t timeout_ms);
bool zed_f9p_set_dyn_model(zed_f9p_t *dev, zed_f9p_dyn_model_t model, zed_f9p_write_fn write_fn, void *context, uint32_t now_ms, uint32_t timeout_ms);

bool zed_f9p_set_baudrate(zed_f9p_t *dev, zed_f9p_port_t port, uint32_t baud, zed_f9p_write_fn write_fn, void *context, uint32_t now_ms, uint32_t timeout_ms);

bool zed_f9p_start_rtk_rover(zed_f9p_t *dev, zed_f9p_port_t port, zed_f9p_port_t rtcm_port, zed_f9p_write_fn write_fn, void *context, uint32_t now_ms, uint32_t timeout_ms);


bool zed_f9p_start_base_survey(zed_f9p_t *dev, zed_f9p_port_t port, uint32_t min_dur_s, uint32_t acc_limit_1e1_mm, zed_f9p_write_fn write_fn, void *context, uint32_t now_ms, uint32_t timeout_ms);

//height above ellipsoid
bool zed_f9p_start_base_fixed(zed_f9p_t *dev, zed_f9p_port_t port, int64_t lat_1e9_deg, int64_t lon_1e9_deg, int64_t height_1e1_mm, uint32_t acc_1e1_mm, zed_f9p_write_fn write_fn, void *context, uint32_t now_ms, uint32_t timeout_ms);

void zed_f9p_rtcm_init(zed_f9p_rtcm_t *rtcm);

bool zed_f9p_rtcm_feed(zed_f9p_rtcm_t *rtcm, uint8_t byte);
size_t zed_f9p_rtcm_length(const zed_f9p_rtcm_t *rtcm); //whole frame incl header + crc
uint16_t zed_f9p_rtcm_type(const zed_f9p_rtcm_t *rtcm); //message number, eg 1005
#endif
