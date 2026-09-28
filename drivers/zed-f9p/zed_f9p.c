#include "zed_f9p.h"
#include <string.h>
#define UBX_SYNC1 0xB5u
#define UBX_SYNC2 0x62u
#define UBX_CLASS_NAV 0x01u
#define UBX_CLASS_ACK 0x05u
#define UBX_CLASS_CFG 0x06u
#define UBX_ID_NAV_PVT 0x07u
#define UBX_ID_NAV_HPPOSLLH 0x14u
#define UBX_ID_NAV_SVIN 0x3Bu
#define UBX_ID_NAV_RELPOSNED 0x3Cu
#define UBX_ID_ACK_ACK 0x01u
#define UBX_ID_CFG_VALSET 0x8Au
#define RTCM_PREAMBLE 0xD3u
#define MIN_MEAS_MS 25u
#define PVT_LEN  92u
#define PVT_ITOW  0u
#define PVT_FIXTYPE 20u
#define PVT_FLAGS 21u
#define PVT_NUMSV 23u
#define PVT_LON 24u
#define PVT_LAT 28u
#define PVT_HEIGHT 32u
#define PVT_HMSL 36u
#define PVT_HACC 40u
#define PVT_VACC 44u
#define PVT_VELN 48u
#define PVT_VELE  52u
#define PVT_VELD 56u
#define PVT_FLAG_FIX_OK 0x01u
#define PVT_FLAG_DIFFSOLN 0x02u
#define PVT_FLAGS3 78u
#define HPPOSLLH_LEN 36u
#define RELPOSNED_LEN 64u
#define RELPOSNED_VERSION 1u
#define SVIN_LEN 40u

static uint16_t read_u16(const uint8_t *p) {
    return (uint16_t)((uint16_t)p[0] | (uint16_t)((uint16_t)p[1] << 8));
}
static uint32_t read_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8)| ((uint32_t)p[2] << 16)| ((uint32_t)p[3] << 24);
}
//low byte first
static int32_t read_i32(const uint8_t *p) {
    const uint32_t value = read_u32(p);
    return value <= INT32_MAX ? (int32_t)value : -1 - (int32_t)(UINT32_MAX - value);
}
static int8_t read_i8(uint8_t byte) {
    return (int8_t)(byte <= INT8_MAX ? (int)byte : (int)byte - 256);
}

static int64_t read_hp(const uint8_t *p, uint8_t hp, int64_t scale) {
    return (int64_t)read_i32(p) * scale + read_i8(hp);
}

static void accumulate(zed_f9p_t *dev, uint8_t byte) {
    dev->ck_a = (uint8_t)(dev->ck_a + byte);
    dev->ck_b = (uint8_t)(dev->ck_b + dev->ck_a);
}

static void decode_pvt(zed_f9p_t *dev) {
    const uint8_t *p = dev->payload;
    zed_f9p_pvt_t *o = &dev->pvt;

    o->year = read_u16(p + 4);
    o->month = p[6]; o->day = p[7]; o->hour = p[8];
    o->minute = p[9]; o->second = p[10]; o->valid = p[11];
    o->time_acc_ns = read_u32(p + 12); o->nano = read_i32(p + 16);
    o->flags = p[21]; o->flags2 = p[22]; o->flags3 = read_u16(p + 78);
    o->ground_speed_mms = read_i32(p + 60);
    o->heading_motion_1e5_deg = read_i32(p + 64);
    o->speed_acc_mms = read_u32(p + 68);
    o->heading_acc_1e5_deg = read_u32(p + 72);
    o->pdop_1e2 = read_u16(p + 76);
    o->heading_vehicle_1e5_deg = read_i32(p + 84);
    const uint16_t mag = read_u16(p + 88);
    o->magnetic_declination_1e2_deg = (int16_t)(mag <= INT16_MAX ? (int32_t)mag : (int32_t)mag - 65536);
    o->magnetic_acc_1e2_deg = read_u16(p + 90);
    o->itow_ms = read_u32(&p[PVT_ITOW]);
    o->fix_type= p[PVT_FIXTYPE];
    o->num_sats = p[PVT_NUMSV];
    o->fix_ok = (p[PVT_FLAGS] & PVT_FLAG_FIX_OK)   != 0u;
    o->corrections_applied = (p[PVT_FLAGS] & PVT_FLAG_DIFFSOLN) != 0u;
    o->carrier_soln = (uint8_t)((p[PVT_FLAGS] >> 6) & 0x03u);

    const uint16_t flags3 = read_u16(&p[PVT_FLAGS3]);
    o->llh_valid = (flags3 & 0x0001u) == 0u;
    o->correction_age = (uint8_t)((flags3 >> 1) & 0x000Fu);
    o->lon_1e7_deg = read_i32(&p[PVT_LON]);
    o->lat_1e7_deg = read_i32(&p[PVT_LAT]);
    o->height_mm = read_i32(&p[PVT_HEIGHT]);
    o->hmsl_mm   = read_i32(&p[PVT_HMSL]);
    o->h_acc_mm = read_u32(&p[PVT_HACC]);
    o->v_acc_mm  = read_u32(&p[PVT_VACC]);
    o->vel_n_mms = read_i32(&p[PVT_VELN]);
    o->vel_e_mms = read_i32(&p[PVT_VELE]);
    o->vel_d_mms = read_i32(&p[PVT_VELD]);

    dev->pvt_valid = true;
    dev->pvt_sequence++;
}

static void decode_hpposllh(zed_f9p_t *dev) {
    const uint8_t *p = dev->payload;
    zed_f9p_hpposllh_t *o = &dev->hpposllh;

    o->llh_valid = (p[3] & 0x01u) == 0u;
    o->itow_ms = read_u32(p + 4);
    o->lon_1e9_deg = read_hp(p + 8, p[24], 100);
    o->lat_1e9_deg = read_hp(p + 12, p[25], 100);
    o->height_1e1_mm = read_hp(p + 16, p[26], 10);
    o->hmsl_1e1_mm = read_hp(p + 20, p[27], 10);
    o->h_acc_1e1_mm = read_u32(p + 28);
    o->v_acc_1e1_mm = read_u32(p + 32);

    dev->hpposllh_valid = true;
    dev->hpposllh_sequence++;
}

static void decode_relposned(zed_f9p_t *dev) {
    const uint8_t *p = dev->payload;
    zed_f9p_relposned_t *o = &dev->relposned;

    o->ref_station_id = read_u16(p + 2);
    o->itow_ms = read_u32(p + 4);
    o->rel_n_1e1_mm = read_hp(p + 8, p[32], 100);
    o->rel_e_1e1_mm = read_hp(p + 12, p[33], 100);
    o->rel_d_1e1_mm = read_hp(p + 16, p[34], 100);
    o->length_1e1_mm = read_hp(p + 20, p[35], 100);
    o->heading_1e5_deg = read_i32(p + 24);
    o->acc_n_1e1_mm = read_u32(p + 36);
    o->acc_e_1e1_mm = read_u32(p + 40);
    o->acc_d_1e1_mm = read_u32(p + 44);
    o->acc_length_1e1_mm = read_u32(p + 48);
    o->acc_heading_1e5_deg = read_u32(p + 52);

    const uint32_t flags = read_u32(p + 60);
    o->flags = flags;
    o->fix_ok = (flags & 0x001u) != 0u;
    o->corrections_applied = (flags & 0x002u) != 0u;
    o->rel_pos_valid = (flags & 0x004u) != 0u;
    o->carrier_soln = (uint8_t)((flags >> 3) & 0x03u);
    o->heading_valid = (flags & 0x100u) != 0u;

    dev->relposned_valid = true;
    dev->relposned_sequence++;
}

static void decode_svin(zed_f9p_t *dev) {
    const uint8_t *p = dev->payload;
    zed_f9p_svin_t *o = &dev->svin;

    o->itow_ms = read_u32(p + 4);
    o->dur_s = read_u32(p + 8);
    o->mean_x_1e1_mm = read_hp(p + 12, p[24], 100);
    o->mean_y_1e1_mm = read_hp(p + 16, p[25], 100);
    o->mean_z_1e1_mm = read_hp(p + 20, p[26], 100);
    o->mean_acc_1e1_mm = read_u32(p + 28);
    o->obs = read_u32(p + 32);
    o->valid = p[36] != 0u;
    o->active = p[37] != 0u;

    dev->svin_valid = true;
    dev->svin_sequence++;
}

static void handle_nav(zed_f9p_t *dev) {
    const uint16_t len = dev->payload_len;
    switch (dev->msg_id) {
    case UBX_ID_NAV_PVT:
        if (len == PVT_LEN) decode_pvt(dev);
        else dev->malformed_messages++;
        break;
    case UBX_ID_NAV_HPPOSLLH:
        if (len == HPPOSLLH_LEN) decode_hpposllh(dev);
        else dev->malformed_messages++;
        break;
    case UBX_ID_NAV_RELPOSNED:
        if (len == RELPOSNED_LEN && dev->payload[0] == RELPOSNED_VERSION) decode_relposned(dev);
        else dev->malformed_messages++;
        break;
    case UBX_ID_NAV_SVIN:
        if (len == SVIN_LEN) decode_svin(dev);
        else dev->malformed_messages++;
        break;
    default:
        break;
    }
}

static void handle_ack(zed_f9p_t *dev) {
    if (dev->payload_len != 2u) {
        dev->malformed_messages++;
    } else if (dev->command_status == ZED_F9P_COMMAND_WAITING && dev->payload[0] == dev->command_class && dev->payload[1] == dev->command_id) {
        dev->command_status = dev->msg_id == UBX_ID_ACK_ACK
            ? ZED_F9P_COMMAND_ACK : ZED_F9P_COMMAND_NAK;
    }
}

void zed_f9p_checksum(const uint8_t *bytes, size_t length, uint8_t *ck_a, uint8_t *ck_b){
    if (ck_a == NULL || ck_b == NULL || (bytes == NULL && length != 0u)) return;
    uint8_t a = 0;
    uint8_t b = 0;

    for (size_t i = 0; i < length; i++) {
        a = (uint8_t)(a + bytes[i]);
        b = (uint8_t)(b + a);
    }

    *ck_a = a;
    *ck_b = b;
}

void zed_f9p_init(zed_f9p_t *dev) {
    if (dev == NULL) return;
    memset(dev, 0, sizeof(*dev));
    dev->state = ZED_F9P_WAIT_SYNC1;
}
bool zed_f9p_feed(zed_f9p_t *dev, uint8_t byte)
{
    if (dev == NULL) return false;
    switch(dev->state){
        case ZED_F9P_WAIT_SYNC1:
            if(byte == UBX_SYNC1)
            {
                dev->state = ZED_F9P_WAIT_SYNC2;
            }
            break;
        case ZED_F9P_WAIT_SYNC2:

        if (byte == UBX_SYNC2) {
            dev->state = ZED_F9P_WAIT_CLASS;
        } else if (byte == UBX_SYNC1) {
            //Stay
        } else {
            dev->state = ZED_F9P_WAIT_SYNC1;
        }
        break;

    case ZED_F9P_WAIT_CLASS:
        dev->ck_a = 0;
        dev->ck_b = 0;
        accumulate(dev, byte);
        dev->msg_class = byte;
        dev->state = ZED_F9P_WAIT_ID;
        break;

    case ZED_F9P_WAIT_ID:
        accumulate(dev, byte);
        dev->msg_id = byte;
        dev->state = ZED_F9P_WAIT_LEN_LO;
        break;

    case ZED_F9P_WAIT_LEN_LO:
        accumulate(dev, byte);
        dev->payload_len = byte;
        dev->state = ZED_F9P_WAIT_LEN_HI;
        break;

    case ZED_F9P_WAIT_LEN_HI:
        accumulate(dev, byte);
        dev->payload_len = (uint16_t)(dev->payload_len | (uint16_t)((uint16_t)byte << 8));
        if (dev->payload_len > ZED_F9P_MAX_PAYLOAD) {
            dev->length_rejects++;
            dev->state = ZED_F9P_WAIT_SYNC1;
        } else if (dev->payload_len == 0u) {
            dev->state = ZED_F9P_WAIT_CK_A;
        } else {
            dev->payload_pos = 0;
            dev->state = ZED_F9P_WAIT_PAYLOAD;
        }
        break;

    case ZED_F9P_WAIT_PAYLOAD:
        accumulate(dev, byte);
        if (dev->payload_pos < ZED_F9P_MAX_PAYLOAD)
            dev->payload[dev->payload_pos] = byte;
        dev->payload_pos++;
        if (dev->payload_pos >= dev->payload_len) {
            dev->state = ZED_F9P_WAIT_CK_A;
        }
        break;

    case ZED_F9P_WAIT_CK_A:
        dev->rx_ck_a = byte;
        dev->state = ZED_F9P_WAIT_CK_B;
        break;

    case ZED_F9P_WAIT_CK_B:
        dev->state = ZED_F9P_WAIT_SYNC1;

        if ((dev->rx_ck_a == dev->ck_a) && (byte == dev->ck_b)) {
            dev->frames_ok++;

            if (dev->msg_class == UBX_CLASS_NAV) {
                handle_nav(dev);
            } else if (dev->msg_class == UBX_CLASS_ACK && dev->msg_id <= UBX_ID_ACK_ACK) {
                handle_ack(dev);
            }
            return true;
        }

        dev->checksum_errors++;

        if (dev->rx_ck_a == UBX_SYNC1 && byte == UBX_SYNC2)
            dev->state = ZED_F9P_WAIT_CLASS;
        else if (byte == UBX_SYNC1) dev->state = ZED_F9P_WAIT_SYNC2;
        break;

    default:
        dev->state = ZED_F9P_WAIT_SYNC1;
        break;
    }
    return false;
}

bool zed_f9p_get_pvt(const zed_f9p_t *dev, zed_f9p_pvt_t *out) {
    if (dev == NULL || out == NULL || !dev->pvt_valid) {
        return false;
    }
    *out = dev->pvt;
    return true;
}

bool zed_f9p_get_hpposllh(const zed_f9p_t *dev, zed_f9p_hpposllh_t *out) {
    if (dev == NULL || out == NULL || !dev->hpposllh_valid) return false;
    *out = dev->hpposllh;
    return true;
}

bool zed_f9p_get_relposned(const zed_f9p_t *dev, zed_f9p_relposned_t *out) {
    if (dev == NULL || out == NULL || !dev->relposned_valid) return false;
    *out = dev->relposned;
    return true;
}

bool zed_f9p_get_svin(const zed_f9p_t *dev, zed_f9p_svin_t *out) {
    if (dev == NULL || out == NULL || !dev->svin_valid) return false;
    *out = dev->svin;
    return true;
}

uint32_t zed_f9p_pvt_age_ms(const zed_f9p_t *dev, uint32_t now_ms) {
    if (dev == NULL || !dev->pvt_valid) return UINT32_MAX;
    //new one since last tick
    if (dev->pvt_sequence != dev->pvt_seen_sequence) return 0;
    return now_ms - dev->pvt_rx_ms;
}

void zed_f9p_reset_parser(zed_f9p_t *dev) {
    if (dev == NULL) return;
    dev->state = ZED_F9P_WAIT_SYNC1;
    dev->payload_pos = 0;
    dev->payload_len = 0;
}

size_t zed_f9p_feed_buffer(zed_f9p_t *dev, const uint8_t *bytes, size_t length) {
    if (dev == NULL || (bytes == NULL && length != 0u)) return 0;
    size_t frames = 0;
    for (size_t i = 0; i < length; ++i) {
        if (zed_f9p_feed(dev, bytes[i])) ++frames;
    }
    return frames;
}

bool zed_f9p_pvt_has_position(const zed_f9p_pvt_t *pvt) {
    return pvt != NULL && pvt->fix_ok && pvt->llh_valid &&
           (pvt->fix_type == ZED_F9P_FIX_3D || pvt->fix_type == ZED_F9P_FIX_GNSS_DR);
}

bool zed_f9p_pvt_has_time(const zed_f9p_pvt_t *pvt) {
    const uint8_t need = ZED_F9P_VALID_DATE | ZED_F9P_VALID_TIME | ZED_F9P_VALID_FULLY_RESOLVED;
    return pvt != NULL && (pvt->valid & need) == need;
}

size_t zed_f9p_encode(uint8_t cls, uint8_t id, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
    if (out == NULL || (payload == NULL && length != 0u) || length > UINT16_MAX || capacity < length + 8u) return 0;
    out[0] = UBX_SYNC1; out[1] = UBX_SYNC2;
    out[2] = cls; out[3] = id;
    out[4] = (uint8_t)length; out[5] = (uint8_t)(length >> 8);
    if (length != 0u) memcpy(out + 6, payload, length);
    zed_f9p_checksum(out + 2, length + 4u, out + length + 6u, out + length + 7u);
    return length + 8u;
}

void zed_f9p_tick(zed_f9p_t *dev, uint32_t now_ms) {
    if (dev == NULL) return;
    if (dev->pvt_sequence != dev->pvt_seen_sequence) {
        dev->pvt_seen_sequence = dev->pvt_sequence;
        dev->pvt_rx_ms = now_ms;
    }

    const uint32_t elapsed = now_ms - dev->command_started_ms;
    if (dev->command_status == ZED_F9P_COMMAND_WAITING && elapsed <= INT32_MAX && elapsed >= dev->command_timeout_ms)
        dev->command_status = ZED_F9P_COMMAND_TIMEOUT;
}

zed_f9p_command_status_t zed_f9p_command_status(const zed_f9p_t *dev) {
    return dev == NULL ? ZED_F9P_COMMAND_IDLE : dev->command_status;
}

bool zed_f9p_configure(zed_f9p_t *dev, const zed_f9p_cfg_t *items, size_t count, zed_f9p_write_fn write_fn, void *context, uint32_t now_ms, uint32_t timeout_ms) {
    if (dev == NULL || items == NULL || count == 0u || write_fn == NULL || timeout_ms == 0u || timeout_ms > INT32_MAX || dev->command_status == ZED_F9P_COMMAND_WAITING) return false;
    uint8_t payload[ZED_F9P_MAX_PAYLOAD] = {0, 1, 0, 0}; //v0, RAM only
    size_t pos = 4;
    for (size_t i = 0; i < count; ++i) {
        const uint32_t type = items[i].key >> 28;
        if (type < 1u || type > 5u) return false;
        const size_t size = type == 1u ? 1u : (size_t)1u << (type - 2u);
        if ((type == 1u && items[i].value > 1u) ||
            (size < 8u && (items[i].value >> (size * 8u)) != 0u) ||
            pos + 4u + size > sizeof(payload)) return false;
        for (size_t j = 0; j < 4u; ++j)
            payload[pos++] = (uint8_t)(items[i].key >> (j * 8u));
        for (size_t j = 0; j < size; ++j)
            payload[pos++] = (uint8_t)(items[i].value >> (j * 8u));
    }
    uint8_t frame[ZED_F9P_MAX_PAYLOAD + 8u];
    const size_t length = zed_f9p_encode(UBX_CLASS_CFG, UBX_ID_CFG_VALSET, payload, pos, frame, sizeof(frame));
    dev->command_class = UBX_CLASS_CFG; dev->command_id = UBX_ID_CFG_VALSET;
    dev->command_started_ms = now_ms; dev->command_timeout_ms = timeout_ms;
    dev->command_status = ZED_F9P_COMMAND_WAITING;
    if (!write_fn(context, frame, length)) {
        dev->command_status = ZED_F9P_COMMAND_IO_ERROR;
        return false;
    }
    return true;
}

static bool valid_port(zed_f9p_port_t port) {
    return (unsigned)port <= (unsigned)ZED_F9P_PORT_SPI;
}

bool zed_f9p_start_pvt(zed_f9p_t *dev, zed_f9p_port_t port, uint16_t period_ms, zed_f9p_write_fn write_fn, void *context, uint32_t now_ms, uint32_t timeout_ms) {
    //f9p naks anything under 25 ms and then nothing gets applied
    if (!valid_port(port) || period_ms < MIN_MEAS_MS) return false;
    const zed_f9p_cfg_t items[] = {
        {ZED_F9P_KEY_OUTPROT(port, ZED_F9P_PROT_UBX), 1},
        {ZED_F9P_KEY_OUTPROT(port, ZED_F9P_PROT_NMEA), 0}, // nmea eats the bandwidth
        {ZED_F9P_KEY_MSGOUT(ZED_F9P_MSGOUT_NAV_PVT, port), 1}, // NAV-PVT output every navigation epoch
        {ZED_F9P_KEY_RATE_MEAS, period_ms},
        {ZED_F9P_KEY_RATE_NAV, 1}
    };
    return zed_f9p_configure(dev, items, sizeof(items) / sizeof(items[0]), write_fn, context, now_ms, timeout_ms);
}

bool zed_f9p_set_dyn_model(zed_f9p_t *dev, zed_f9p_dyn_model_t model, zed_f9p_write_fn write_fn, void *context, uint32_t now_ms, uint32_t timeout_ms) {
    const zed_f9p_cfg_t item = {ZED_F9P_KEY_NAVSPG_DYNMODEL, (uint64_t)model};
    return zed_f9p_configure(dev, &item, 1, write_fn, context, now_ms, timeout_ms);
}

bool zed_f9p_set_baudrate(zed_f9p_t *dev, zed_f9p_port_t port, uint32_t baud, zed_f9p_write_fn write_fn, void *context, uint32_t now_ms, uint32_t timeout_ms) {
    if ((port != ZED_F9P_PORT_UART1 && port != ZED_F9P_PORT_UART2) || baud == 0u) return false;
    const zed_f9p_cfg_t item = {port == ZED_F9P_PORT_UART1 ? ZED_F9P_KEY_UART1_BAUDRATE : ZED_F9P_KEY_UART2_BAUDRATE, baud};
    return zed_f9p_configure(dev, &item, 1, write_fn, context, now_ms, timeout_ms);
}

bool zed_f9p_start_rtk_rover(zed_f9p_t *dev, zed_f9p_port_t port, zed_f9p_port_t rtcm_port, zed_f9p_write_fn write_fn, void *context, uint32_t now_ms, uint32_t timeout_ms) {
    if (!valid_port(port) || !valid_port(rtcm_port)) return false;
    const zed_f9p_cfg_t items[] = {
        {ZED_F9P_KEY_INPROT(rtcm_port, ZED_F9P_PROT_RTCM3), 1},
        {ZED_F9P_KEY_NAVHPG_DGNSSMODE, 3},
        {ZED_F9P_KEY_MSGOUT(ZED_F9P_MSGOUT_NAV_HPPOSLLH, port), 1},
        {ZED_F9P_KEY_MSGOUT(ZED_F9P_MSGOUT_NAV_RELPOSNED, port), 1}
    };
    return zed_f9p_configure(dev, items, sizeof(items) / sizeof(items[0]), write_fn, context, now_ms, timeout_ms);
}

static size_t add_base_rtcm(zed_f9p_cfg_t *items, zed_f9p_port_t port) {
    static const uint32_t msgs[] = {
        ZED_F9P_MSGOUT_RTCM_1005, ZED_F9P_MSGOUT_RTCM_1074, ZED_F9P_MSGOUT_RTCM_1084,
        ZED_F9P_MSGOUT_RTCM_1094, ZED_F9P_MSGOUT_RTCM_1124, ZED_F9P_MSGOUT_RTCM_1230
    };
    size_t n = 0;
    items[n++] = (zed_f9p_cfg_t){ZED_F9P_KEY_OUTPROT(port, ZED_F9P_PROT_RTCM3), 1};
    for (size_t i = 0; i < sizeof(msgs) / sizeof(msgs[0]); ++i)
        items[n++] = (zed_f9p_cfg_t){ZED_F9P_KEY_MSGOUT(msgs[i], port), 1};
    return n;
}

bool zed_f9p_start_base_survey(zed_f9p_t *dev, zed_f9p_port_t port, uint32_t min_dur_s, uint32_t acc_limit_1e1_mm, zed_f9p_write_fn write_fn, void *context, uint32_t now_ms, uint32_t timeout_ms) {
    if (!valid_port(port) || min_dur_s == 0u || acc_limit_1e1_mm == 0u) return false;
    zed_f9p_cfg_t items[11];
    size_t n = add_base_rtcm(items, port);
    items[n++] = (zed_f9p_cfg_t){ZED_F9P_KEY_MSGOUT(ZED_F9P_MSGOUT_NAV_SVIN, port), 1};
    items[n++] = (zed_f9p_cfg_t){ZED_F9P_KEY_TMODE_MODE, 1};
    items[n++] = (zed_f9p_cfg_t){ZED_F9P_KEY_TMODE_SVIN_MIN_DUR, min_dur_s};
    items[n++] = (zed_f9p_cfg_t){ZED_F9P_KEY_TMODE_SVIN_ACC_LIMIT, acc_limit_1e1_mm};
    return zed_f9p_configure(dev, items, n, write_fn, context, now_ms, timeout_ms);
}

bool zed_f9p_start_base_fixed(zed_f9p_t *dev, zed_f9p_port_t port, int64_t lat_1e9_deg, int64_t lon_1e9_deg, int64_t height_1e1_mm, uint32_t acc_1e1_mm, zed_f9p_write_fn write_fn, void *context, uint32_t now_ms, uint32_t timeout_ms) {
    if (!valid_port(port) ||
        lat_1e9_deg < -90000000000 || lat_1e9_deg > 90000000000 ||
        lon_1e9_deg < -180000000000 || lon_1e9_deg > 180000000000 ||
        height_1e1_mm / 100 < INT32_MIN || height_1e1_mm / 100 > INT32_MAX) return false;
    const int32_t lat = (int32_t)(lat_1e9_deg / 100), lon = (int32_t)(lon_1e9_deg / 100);
    const int32_t height_cm = (int32_t)(height_1e1_mm / 100);
    const int8_t lat_hp = (int8_t)(lat_1e9_deg % 100), lon_hp = (int8_t)(lon_1e9_deg % 100);
    const int8_t height_hp = (int8_t)(height_1e1_mm % 100);

    zed_f9p_cfg_t items[16];
    size_t n = add_base_rtcm(items, port);
    items[n++] = (zed_f9p_cfg_t){ZED_F9P_KEY_TMODE_MODE, 2};
    items[n++] = (zed_f9p_cfg_t){ZED_F9P_KEY_TMODE_POS_TYPE, 1};
    items[n++] = (zed_f9p_cfg_t){ZED_F9P_KEY_TMODE_LAT, (uint32_t)lat};
    items[n++] = (zed_f9p_cfg_t){ZED_F9P_KEY_TMODE_LON, (uint32_t)lon};
    items[n++] = (zed_f9p_cfg_t){ZED_F9P_KEY_TMODE_HEIGHT, (uint32_t)height_cm};
    items[n++] = (zed_f9p_cfg_t){ZED_F9P_KEY_TMODE_LAT_HP, (uint8_t)lat_hp};
    items[n++] = (zed_f9p_cfg_t){ZED_F9P_KEY_TMODE_LON_HP, (uint8_t)lon_hp};
    items[n++] = (zed_f9p_cfg_t){ZED_F9P_KEY_TMODE_HEIGHT_HP, (uint8_t)height_hp};
    items[n++] = (zed_f9p_cfg_t){ZED_F9P_KEY_TMODE_FIXED_POS_ACC, acc_1e1_mm};
    return zed_f9p_configure(dev, items, n, write_fn, context, now_ms, timeout_ms);
}

//crc-24q, bitwise so no 1k table
static uint32_t crc24q(const uint8_t *bytes, size_t length) {
    uint32_t crc = 0;
    for (size_t i = 0; i < length; ++i) {
        crc ^= (uint32_t)bytes[i] << 16;
        for (unsigned bit = 0; bit < 8u; ++bit) {
            crc <<= 1;
            if ((crc & 0x1000000u) != 0u) crc ^= 0x1864CFBu;
        }
    }
    return crc & 0xFFFFFFu;
}

void zed_f9p_rtcm_init(zed_f9p_rtcm_t *rtcm) {
    if (rtcm == NULL) return;
    memset(rtcm, 0, sizeof(*rtcm));
}

bool zed_f9p_rtcm_feed(zed_f9p_rtcm_t *rtcm, uint8_t byte) {
    if (rtcm == NULL) return false;
    if (rtcm->pos == 0u && byte != RTCM_PREAMBLE) return false;
    //top 6 bits reserved 0
    if (rtcm->pos == 1u && (byte & 0xFCu) != 0u) {
        rtcm->pos = byte == RTCM_PREAMBLE ? 1u : 0u;
        return false;
    }
    rtcm->frame[rtcm->pos++] = byte;
    if (rtcm->pos == 3u)
        rtcm->length = (uint16_t)(((rtcm->frame[1] & 0x03u) << 8) | rtcm->frame[2]);
    if (rtcm->pos < 3u || rtcm->pos < rtcm->length + 6u) return false;

    rtcm->pos = 0;
    const uint8_t *crc = &rtcm->frame[rtcm->length + 3u];
    const uint32_t rx_crc = ((uint32_t)crc[0] << 16) | ((uint32_t)crc[1] << 8) | crc[2];
    if (crc24q(rtcm->frame, rtcm->length + 3u) != rx_crc) {
        rtcm->crc_errors++;
        return false;
    }
    rtcm->frames_ok++;
    return true;
}

size_t zed_f9p_rtcm_length(const zed_f9p_rtcm_t *rtcm) {
    return rtcm == NULL ? 0u : (size_t)rtcm->length + 6u;
}

uint16_t zed_f9p_rtcm_type(const zed_f9p_rtcm_t *rtcm) {
    if (rtcm == NULL || rtcm->length < 2u) return 0;
    return (uint16_t)((rtcm->frame[3] << 4) | (rtcm->frame[4] >> 4));
}
