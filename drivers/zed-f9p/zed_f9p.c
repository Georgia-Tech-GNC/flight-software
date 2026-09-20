#include "zed_f9p.h"
#include <string.h>
#define UBX_SYNC1 0xB5u
#define UBX_SYNC2 0x62u
#define UBX_CLASS_NAV 0x01u
#define UBX_ID_NAV_PVT 0x07u
//field offsets from interface description
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



void zed_f9p_checksum(const uint8_t *bytes, const size_t length, uint8_t *ck_a, uint8_t *ck_b){
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

        //Consume oversized frames without storing them
        if (dev->payload_len > ZED_F9P_MAX_PAYLOAD) dev->length_rejects++;
        if (dev->payload_len == 0u) {
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

            if ((dev->msg_class == UBX_CLASS_NAV) && (dev->msg_id == UBX_ID_NAV_PVT) && (dev->payload_len == PVT_LEN)) {
                decode_pvt(dev);
            } else if (dev->msg_class == UBX_CLASS_NAV && dev->msg_id == UBX_ID_NAV_PVT) {
                dev->malformed_messages++;
            } else if (dev->msg_class == 0x05u && dev->msg_id <= 1u) {
                if (dev->payload_len != 2u) {
                    dev->malformed_messages++;
                } else if (dev->command_status == ZED_F9P_COMMAND_WAITING && dev->payload[0] == dev->command_class && dev->payload[1] == dev->command_id) {
                    dev->command_status = dev->msg_id == 1u
                        ? ZED_F9P_COMMAND_ACK : ZED_F9P_COMMAND_NAK;
                }
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
           (pvt->fix_type == 2u || pvt->fix_type == 3u || pvt->fix_type == 4u);
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
    if (dev != NULL && dev->command_status == ZED_F9P_COMMAND_WAITING && (uint32_t)(now_ms - dev->command_started_ms) >= dev->command_timeout_ms)
        dev->command_status = ZED_F9P_COMMAND_TIMEOUT;
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
    const size_t length = zed_f9p_encode(0x06, 0x8a, payload, pos, frame, sizeof(frame));
    dev->command_class = 0x06; dev->command_id = 0x8a;
    dev->command_started_ms = now_ms; dev->command_timeout_ms = timeout_ms;
    dev->command_status = ZED_F9P_COMMAND_WAITING;
    if (!write_fn(context, frame, length)) {
        dev->command_status = ZED_F9P_COMMAND_IO_ERROR;
        return false;
    }
    return true;
}

bool zed_f9p_start_pvt(zed_f9p_t *dev, zed_f9p_port_t port, uint16_t period_ms, zed_f9p_write_fn write_fn, void *context, uint32_t now_ms, uint32_t timeout_ms) {

    static const uint32_t output_keys[] = {
        0x10720001u, 0x10740001u, 0x10760001u, 0x10780001u, 0x107a0001u
    };
    if ((unsigned)port > (unsigned)ZED_F9P_PORT_SPI || period_ms == 0u) return false;
    const zed_f9p_cfg_t items[] = {
        {output_keys[port], 1},
        {0x20910006u + (uint32_t)port, 1}, // NAV-PVT output every navigation epoch
        {0x30210001u, period_ms},
        {0x30210002u, 1}
    };
    return zed_f9p_configure(dev, items, sizeof(items) / sizeof(items[0]), write_fn, context, now_ms, timeout_ms);
}
