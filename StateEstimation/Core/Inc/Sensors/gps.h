#ifndef __UBLOX_GNSS_H__
#define __UBLOX_GNSS_H__

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "arm_math.h"

#define UBLOX_PROTOCOL_HEADER_LENGTH_BYTES 6

#define UBLOX_PROTOCOL_OVERHEAD_LENGTH_BYTES                                   \
  (UBLOX_PROTOCOL_HEADER_LENGTH_BYTES + 2)

#define UBLOX_GNSS_CFG_VAL_MSG_MAX_NUM_VALUES 64

#define UBLOX_GNSS_CFG_VAL_KEY_GROUP_ID_ALL 0xFFF

#define UBLOX_GNSS_CFG_VAL_KEY_ITEM_ID_ALL 0xFFFF

#define UBLOX_GNSS_DEC_UBX_NAV_TIMEUTC_BODY_LENGTH 20
#define UBLOX_GNSS_DEC_UBX_NAV_COV_BODY_LENGTH     64
#define UBLOX_GNSS_DEC_UBX_NAV_POSECEF_BODY_LENGTH 20
#define UBLOX_GNSS_DEC_UBX_NAV_POSLLH_BODY_LENGTH  28
#define UBLOX_GNSS_DEC_UBX_NAV_PVT_BODY_LENGTH 92

#define UBLOX_GNSS_CFG_VAL_KEY_GET_ITEM_ID(key_id)                             \
  (((uint32_t)(key_id)) & 0xFFFF)

#define UBLOX_GNSS_CFG_VAL_KEY_GET_GROUP_ID(key_id)                            \
  ((((uint32_t)(key_id)) >> 16) & 0xFFF)

// cast to (struct ublox_gnss_cfg_val_key_size)
#define UBLOX_GNSS_CFG_VAL_KEY_GET_SIZE(key_id)                                \
  (((((uint32_t)(key_id)) >> 28) & 0x07))

#define UBLOX_GNSS_CFG_VAL_KEY(group_id, item_id, size)                        \
  ((((uint32_t)(size)&0x07) << 28) | (((uint32_t)(group_id)&0xFFF) << 16) |    \
   (((uint32_t)(item_id)) & 0xFFFF))

enum ublox_gnss_err {
  UBLOX_GNSS_ERR_OK,
};

enum ublox_gnss_transport_type {
  UBLOX_GNSS_TRANSPORT_NONE,
  UBLOX_GNSS_TRANSPORT_UART,
  UBLOX_GNSS_TRANSPORT_AT,
  UBLOX_GNSS_TRANSPORT_I2C,
  UBLOX_GNSS_TRANSPORT_SPI,
  UBLOX_GNSS_TRANSPORT_VIRTUAL_SERIAL,
  UBLOX_GNSS_TRANSPORT_UART_2,
  UBLOX_GNSS_TRANSPORT_USB,
  UBLOX_GNSS_TRANSPORT_MAX_NUM,
  UBLOX_GNSS_TRANSPORT_UART_1 = UBLOX_GNSS_TRANSPORT_UART,
};

enum ublox_gnss_cfg_val_transaction {
  UBLOX_GNSS_CFG_VAL_TRANSACTION_NONE = 0,
  UBLOX_GNSS_CFG_VAL_TRANSACTION_BEGIN = 1,
  UBLOX_GNSS_CFG_VAL_TRANSACTION_CONTINUE = 2,
  UBLOX_GNSS_CFG_VAL_TRANSACTION_EXECUTE = 3,
  UBLOX_GNSS_CFG_VAL_TRANSACTION_MAX_NUM,
};

enum ublox_gnss_cfg_val_layer {
  UBLOX_GNSS_CFG_VAL_LAYER_NONE = 0x00,
  UBLOX_GNSS_CFG_VAL_LAYER_RAM = 0x01,
  UBLOX_GNSS_CFG_VAL_LAYER_BBRAM = 0x02,
  UBLOX_GNSS_CFG_VAL_LAYER_FLASH = 0x04,
  UBLOX_GNSS_CFG_VAL_LAYER_DEFAULT = 0x07,
  UBLOX_GNSS_CFG_VAL_LAYER_MAX_NUM,
};

enum ublox_gnss_cfg_val_key_size {
  UBLOX_GNSS_CFG_VAL_KEY_SIZE_NONE = 0x00,
  UBLOX_GNSS_CFG_VAL_KEY_SIZE_ONE_BIT = 0x01,
  UBLOX_GNSS_CFG_VAL_KEY_SIZE_ONE_BYTE = 0x02,
  UBLOX_GNSS_CFG_VAL_KEY_SIZE_TWO_BYTES = 0x03,
  UBLOX_GNSS_CFG_VAL_KEY_SIZE_FOUR_BYTES = 0x04,
  UBLOX_GNSS_CFG_VAL_KEY_SIZE_EIGHT_BYTES = 0x05,
};

struct ublox_gnss_cfg_val {
  uint32_t key_id;
  uint64_t value;
};

struct ublox_gnss_nav_timeutc {
  uint32_t itow;
  uint32_t t_acc;
  int32_t nano;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint8_t valid;
};

struct ublox_gnss_nav_cov {
  uint32_t itow;
  uint8_t version;
  uint8_t pos_cov_valid;
  uint8_t vel_cov_valid;
  float32_t pos_cov_nn;
  float32_t pos_cov_ne;
  float32_t pos_cov_nd;
  float32_t pos_cov_ee;
  float32_t pos_cov_ed;
  float32_t pos_cov_dd;
  float32_t vel_cov_nn;
  float32_t vel_cov_ne;
  float32_t vel_cov_nd;
  float32_t vel_cov_ee;
  float32_t vel_cov_ed;
  float32_t vel_cov_dd;
};

struct ublox_gnss_nav_posecef {
  uint32_t itow;
  int32_t ecef_x;
  int32_t ecef_y;
  int32_t ecef_z;
  uint32_t p_acc;
};

struct ublox_gnss_nav_posllh {
  uint32_t itow;
  int32_t lon;
  int32_t lat;
  int32_t height;
  int32_t h_msl;
  uint32_t h_acc;
  uint32_t v_acc;
};

struct ublox_gnss_nav_velned {
  uint32_t itow;
  int32_t vel_n;
  int32_t vel_e;
  int32_t vel_d;
  uint32_t speed;
  uint32_t g_speed;
  int32_t heading;
  uint32_t s_acc;
  uint32_t c_acc;
};

struct ublox_gnss_nav_pvt {
  uint32_t itow;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint8_t valid;
  uint32_t t_acc;
  int32_t nano;
  uint8_t fix_type;
  uint8_t flags;
  uint8_t flags2;
  uint8_t num_sv;
  int32_t lon;
  int32_t lat;
  int32_t height;
  int32_t h_msl;
  uint32_t h_acc;
  uint32_t v_acc;
  int32_t vel_n;
  int32_t vel_e;
  int32_t vel_d;
  int32_t g_speed;
  int32_t head_mot;
  uint32_t s_acc;
  uint32_t head_acc;
  uint16_t p_dop;
  uint16_t flags3;
  int32_t head_veh;
  int16_t mag_dec;
  uint16_t mag_acc;
};

// High Precision Position ECEF struct
struct ublox_gnss_nav_hpposecef {
    uint32_t version;      // Message version (0x00 for this version)
    uint32_t itow;         // GPS time of week
    int32_t ecefX;        // ECEF X coordinate (cm)
    int32_t ecefY;        // ECEF Y coordinate (cm)
    int32_t ecefZ;        // ECEF Z coordinate (cm)
    int8_t ecefXHp;      // High precision component of ECEF X (mm)
    int8_t ecefYHp;      // High precision component of ECEF Y (mm)
    int8_t ecefZHp;      // High precision component of ECEF Z (mm)
    uint8_t reserved1;    // Reserved
    uint32_t pAcc;        // Position Accuracy Estimate (0.1 mm)
};

// High Precision Position, Velocity and Time struct
struct ublox_gnss_nav_hppvt {
    uint32_t itow;         // GPS time of week
    uint16_t year;         // Year (UTC)
    uint8_t month;        // Month (UTC)
    uint8_t day;          // Day of month (UTC)
    uint8_t hour;         // Hour of day (UTC)
    uint8_t min;          // Minute of hour (UTC)
    uint8_t sec;          // Seconds of minute (UTC)
    uint8_t valid;        // Valid flags
    uint32_t tAcc;        // Time accuracy estimate (UTC)
    int32_t nano;         // Fraction of second (ns)
    uint8_t fixType;      // GNSS fix type
    uint8_t flags;        // Fix status flags
    uint8_t flags2;       // Additional flags
    uint8_t numSV;        // Number of satellites used
    int32_t lon;          // Longitude (deg * 1e-7)
    int32_t lat;          // Latitude (deg * 1e-7)
    int32_t height;       // Height above ellipsoid (mm)
    int32_t hMSL;         // Height above mean sea level (mm)
    int8_t lonHp;        // High precision component of longitude (deg * 1e-9)
    int8_t latHp;        // High precision component of latitude (deg * 1e-9)
    int8_t heightHp;     // High precision component of height (0.1 mm)
    int8_t hMSLHp;       // High precision component of hMSL (0.1 mm)
    uint32_t hAcc;        // Horizontal accuracy (0.1 mm)
    uint32_t vAcc;        // Vertical accuracy (0.1 mm)
    int32_t velN;         // NED north velocity (mm/s)
    int32_t velE;         // NED east velocity (mm/s)
    int32_t velD;         // NED down velocity (mm/s)
    int32_t gSpeed;       // Ground Speed (mm/s)
    int32_t headMot;      // Heading of motion (deg * 1e-5)
    uint32_t sAcc;        // Speed accuracy (mm/s)
    uint32_t headAcc;     // Heading accuracy (deg * 1e-5)
    uint16_t pDOP;        // Position DOP * 0.01
    uint8_t flags3;       // Additional flags
    uint8_t reserved1[5]; // Reserved
    int32_t headVeh;      // Heading of vehicle (deg * 1e-5)
    int16_t magDec;       // Magnetic declination (deg * 1e-2)
    uint16_t magAcc;      // Magnetic declination accuracy (deg * 1e-2)
};

#define UBLOX_GNSS_DEC_UBX_NAV_HPPOSECEF_BODY_LENGTH 28
#define UBLOX_GNSS_DEC_UBX_NAV_HPPVT_BODY_LENGTH     68

void ublox_gnss_dec_ubx_nav_hpposecef(uint8_t *msg, uint16_t msg_length_bytes,
                                     struct ublox_gnss_nav_hpposecef *nav_hpposecef);

void ublox_gnss_dec_ubx_nav_hppvt(uint8_t *msg, uint16_t msg_length_bytes,
                                 struct ublox_gnss_nav_hppvt *nav_hppvt);


struct ublox_gnss_nav_sat {
  uint32_t itow;
  uint8_t version;
  uint8_t num_svs;
};

union ublox_gnss_transport_handle {
  void *at;
  void *uart;
  void *i2c;
  void *spi;
  void *serial_device;
};

struct ublox_gnss_device {
  enum ublox_gnss_transport_type transport_type;
  union ublox_gnss_transport_handle transport_handle;
};

struct ublox_gnss_cfg_val_set_msg {};

enum ublox_gnss_err ublox_gnss_send_msg(struct ublox_gnss_device *device,
                                        const uint8_t *buffer, uint16_t size);

uint16_t ublox_protocol_u16_decode(const uint8_t *byte);

uint32_t ublox_protocol_u32_decode(const uint8_t *byte);

uint64_t ublox_protocol_u64_decode(const uint8_t *byte);

uint16_t ublox_protocol_u16_encode(uint16_t u16);

uint32_t ublox_protocol_u32_encode(uint32_t u32);

uint64_t ublox_protocol_u64_encode(uint64_t u64);

float32_t ublox_protocol_f32_decode(const uint8_t *byte);

void ublox_protocol_encode(uint8_t class, uint8_t id, uint8_t *msg,
                           uint16_t msg_length_bytes, uint8_t *buf);

void ublox_protocol_decode(uint8_t *buf, uint16_t buf_length_bytes,
                           uint8_t *class, uint8_t *id, uint8_t *msg,
                           uint16_t max_msg_length_bytes,
                           uint16_t *msg_length_bytes, uint8_t **rem);

void ublox_gnss_cfg_val_set_list(
    struct ublox_gnss_device *device, struct ublox_gnss_cfg_val *list,
    uint16_t number_of_values, enum ublox_gnss_cfg_val_transaction transaction,
    uint32_t layers);

void ublox_gnss_cfg_val_set(struct ublox_gnss_device *device, uint32_t key_id,
                            uint64_t value,
                            enum ublox_gnss_cfg_val_transaction transaction,
                            uint32_t layers);

void ublox_gnss_dec_ubx_nav_timeutc(uint8_t *msg, uint16_t msg_length_bytes,
                                    struct ublox_gnss_nav_timeutc *nav_timeutc);

void ublox_gnss_dec_ubx_nav_cov(uint8_t *msg, uint16_t msg_length_bytes,
                                struct ublox_gnss_nav_cov *nav_cov);

void ublox_gnss_dec_ubx_nav_posecef(uint8_t *msg, uint16_t msg_length_bytes,
                                    struct ublox_gnss_nav_posecef *nav_posecef);

void ublox_gnss_dec_ubx_nav_posllh(uint8_t *msg, uint16_t msg_length_bytes,
                                   struct ublox_gnss_nav_posllh *nav_posllh);

void ublox_gnss_dec_ubx_nav_pvt(uint8_t *msg, uint16_t msg_length_bytes,
                                struct ublox_gnss_nav_pvt *nav_pvt);

#endif /* __UBLOX_GNSS_H__ */