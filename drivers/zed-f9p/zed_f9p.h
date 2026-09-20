#ifndef ZED_F9P_H
#define ZED_F9P_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define ZED_F9P_MAX_PAYLOAD 128u

typedef struct {
    uint16_t year;
    uint8_t month, day, hour, minute, second;
    uint8_t valid, flags, flags2; //validity/fix/time confirmation flags
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
    int32_t  vel_n_mms; //north velocity
    int32_t  vel_e_mms; //east velocity
    int32_t  vel_d_mms; //down velocity
    uint8_t fix_type; // 0 none, 2 two dimensional, 3 three dimensional
    uint8_t num_sats; //satellites used
    bool fix_ok; //receiver considers fix usable
    bool corrections_applied; //differential corrections used
    uint8_t carrier_soln; //0 none, 1 float, 2 fixed
    uint8_t correction_age; //age category
    bool llh_valid; //position fields valid


} zed_f9p_pvt_t;

typedef enum{
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

// Must copy all bytes before returning
typedef bool (*zed_f9p_write_fn)(void *context, const uint8_t *bytes, size_t length);

typedef struct {
    uint32_t key;
    uint64_t value; //raw bit
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
} zed_f9p_t;

/** Computes two-byte checksum for UBX message
 * Uses  8-bit Fletcher algorithm. Checksum covers message class, ID, length field, and payload
 * Excludes the two sync characters at the start and the checksum bytes themselves

 * @param bytes   pointer to the first byte to include, which is the message class
 * @param length  number of bytes to include, equal to the payload length plus four
 * @param ck_a    set to the first checksum byte
 * @param ck_b    set to the second checksum byte
 */

void zed_f9p_checksum(const uint8_t *bytes, const size_t length, uint8_t *ck_a, uint8_t *ck_b);
//Reset driver to starting state
void zed_f9p_init(zed_f9p_t *dev);

/** Feeds one received byte into parser
 * @returns true when this byte completed a message that passed its checksum
 */

bool zed_f9p_feed(zed_f9p_t *dev, uint8_t byte);
/** Copies out most recent navigation solution
 * @returns false if no solution has been decoded yet
 */
bool zed_f9p_get_pvt(const zed_f9p_t *dev, zed_f9p_pvt_t *out);

size_t zed_f9p_feed_buffer(zed_f9p_t *dev, const uint8_t *bytes, size_t length);
// Reset framing after transport error
void zed_f9p_reset_parser(zed_f9p_t *dev);
bool zed_f9p_pvt_has_position(const zed_f9p_pvt_t *pvt);
// Returns total frame length or 0 for invalid args
// Payload and output shouldnt overlap
size_t zed_f9p_encode(uint8_t cls, uint8_t id, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);

bool zed_f9p_configure(zed_f9p_t *dev, const zed_f9p_cfg_t *items, size_t count,  zed_f9p_write_fn write_fn, void *context,uint32_t now_ms, uint32_t timeout_ms);
bool zed_f9p_start_pvt(zed_f9p_t *dev, zed_f9p_port_t port, uint16_t period_ms,zed_f9p_write_fn write_fn, void *context, uint32_t now_ms, uint32_t timeout_ms);
void zed_f9p_tick(zed_f9p_t *dev, uint32_t now_ms);
#endif
