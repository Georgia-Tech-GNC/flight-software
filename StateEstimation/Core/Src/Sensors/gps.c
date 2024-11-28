#include "gps.h"


static __attribute__((always_inline)) inline uint16_t
cfg_val_key_get_storage_size_bytes(
    enum ublox_gnss_cfg_val_key_size storage_size) {
  uint16_t size = 0;

  switch (storage_size) {
  case UBLOX_GNSS_CFG_VAL_KEY_SIZE_ONE_BIT:
  case UBLOX_GNSS_CFG_VAL_KEY_SIZE_ONE_BYTE:
    size = 1;
    break;
  case UBLOX_GNSS_CFG_VAL_KEY_SIZE_TWO_BYTES:
    size = 2;
    break;
  case UBLOX_GNSS_CFG_VAL_KEY_SIZE_FOUR_BYTES:
    size = 4;
    break;
  case UBLOX_GNSS_CFG_VAL_KEY_SIZE_EIGHT_BYTES:
    size = 8;
    break;
  default:
    break;
  }

  return size;
}

bool ublox_protocol_is_little_endian() {
  uint32_t x = 1;
  return (*((uint8_t *)(&x)) == 1);
}

uint16_t ublox_protocol_u16_decode(const uint8_t *byte) {
  uint16_t ret;

  ret = (uint16_t)(*byte) | (((uint16_t) * (byte + 1)) << 8);

  return ret;
}

uint32_t ublox_protocol_u32_decode(const uint8_t *byte) {
  uint32_t ret;

  ret = (uint32_t)(*byte);
  ret |= ((uint32_t) * (byte + 1)) << 8;
  ret |= ((uint32_t) * (byte + 2)) << 16;
  ret |= ((uint32_t) * (byte + 3)) << 24;

  return ret;
}

uint64_t ublox_protocol_u64_decode(const uint8_t *byte) {
  uint64_t ret;

  ret = (uint64_t)(*byte);
  ret |= ((uint64_t) * (byte + 1)) << 8;
  ret |= ((uint64_t) * (byte + 2)) << 16;
  ret |= ((uint64_t) * (byte + 3)) << 24;
  ret |= ((uint64_t) * (byte + 4)) << 32;
  ret |= ((uint64_t) * (byte + 5)) << 40;
  ret |= ((uint64_t) * (byte + 6)) << 48;
  ret |= ((uint64_t) * (byte + 7)) << 56;

  return ret;
}

float32_t ublox_protocol_f32_decode(const uint8_t *byte) {
  uint32_t tmp;

  tmp = ublox_protocol_u32_decode(byte);

  return *((float32_t *)&tmp);
}

uint16_t ublox_protocol_u16_encode(uint16_t u16) {
  uint16_t ret = u16;

  if (!ublox_protocol_is_little_endian()) {
    ret = (u16 & 0xff00) >> 8;
    ret |= (u16 & 0x00ff) << 8;
  }

  return ret;
}

uint32_t ublox_protocol_u32_encode(uint32_t u32) {
  uint32_t ret = u32;

  if (!ublox_protocol_is_little_endian()) {
    ret = (u32 & 0xff000000) >> 24;
    ret |= (u32 & 0x00ff0000) >> 8;
    ret |= (u32 & 0x0000ff00) << 8;
    ret |= (u32 & 0x000000ff) << 24;
  }

  return ret;
}

uint64_t ublox_protocol_u64_encode(uint64_t u64) {
  uint64_t ret = u64;

  if (!ublox_protocol_is_little_endian()) {
    ret = (u64 & 0xff00000000000000) >> 56;
    ret = (u64 & 0x00ff000000000000) >> 40;
    ret = (u64 & 0x0000ff0000000000) >> 24;
    ret = (u64 & 0x000000ff00000000) >> 8;
    ret = (u64 & 0x00000000ff000000) << 8;
    ret = (u64 & 0x0000000000ff0000) << 24;
    ret = (u64 & 0x000000000000ff00) << 40;
    ret = (u64 & 0x00000000000000ff) << 56;
  }

  return ret;
}

void ublox_protocol_encode(uint8_t class, uint8_t id, uint8_t *msg,
                           uint16_t msg_length_bytes, uint8_t *buf) {
  uint8_t *tmp = buf;
  uint8_t ck_a = 0;
  uint8_t ck_b = 0;
  
  if (((msg_length_bytes == 0) || (msg != NULL)) && (buf != NULL)) {
    *(tmp++) = 0xb5;
    *(tmp++) = 0x62;
    *(tmp++) = class;
    *(tmp++) = id;
    *(tmp++) = (uint8_t)(msg_length_bytes & 0xff);
    *(tmp++) = (uint8_t)((msg_length_bytes & 0xff00) >> 8);
    
    if (msg != NULL) {
      memcpy(tmp, msg, msg_length_bytes);
      tmp += msg_length_bytes;
    }

    buf += 2;

    for (uint16_t i = 0; i < msg_length_bytes + 4; i++) {
      ck_a += buf[i];
      ck_b += ck_a;
    }

    *(tmp++) = ck_a;
    *(tmp) = ck_b;
  }
}

void ublox_protocol_decode(uint8_t *buf, uint16_t buf_length_bytes,
                           uint8_t *class, uint8_t *id, uint8_t *msg,
                           uint16_t max_msg_length_bytes,
                           uint16_t *msg_length_bytes, uint8_t **rem) {
  uint8_t *in = buf;
  uint16_t overhead_byte_count = 0;
  bool update_crc = false;
  uint16_t expected_msg_byte_count = 0;
  uint16_t msg_byte_count = 0;
  uint8_t ck_a = 0;
  uint8_t ck_b = 0;

  for (uint16_t i = 0;
       (i < buf_length_bytes) &&
       (overhead_byte_count < UBLOX_PROTOCOL_OVERHEAD_LENGTH_BYTES);
       i++) {
    switch (overhead_byte_count) {
    case 0:
      if (*in == 0xb5) {
        overhead_byte_count++;
      } else {
        overhead_byte_count = 0;
      }
      break;
    case 1:
      if (*in == 0x62) {
        overhead_byte_count++;
      } else {
        overhead_byte_count = 0;
      }
      break;
    case 2:
      if (class != NULL) {
        *class = *in;
      }
      ck_a = 0;
      ck_b = 0;
      update_crc = true;
      overhead_byte_count++;
      break;
    case 3:
      if (id != NULL) {
        *id = *in;
      }
      update_crc = true;
      overhead_byte_count++;
      break;
    case 4:
      expected_msg_byte_count = *in;
      update_crc = true;
      overhead_byte_count++;
      break;
    case 5:
      expected_msg_byte_count |= ((uint16_t)*in) << 8;
      msg_byte_count = 0;
      update_crc = true;
      overhead_byte_count++;
      break;
    case 6:
      if ((msg != NULL) && (msg_byte_count < expected_msg_byte_count)) {
        if (msg_byte_count < max_msg_length_bytes) {
          *(msg++) = *in;
        }
        update_crc = true;
        msg_byte_count++;
      } else {
        if (ck_a == *in) {
          overhead_byte_count++;
        } else {
          overhead_byte_count = 0;
        }
      }
      break;
    case 7:
      if (ck_b == *in) {
        overhead_byte_count++;
      } else {
        overhead_byte_count = 0;
      }
      break;
    default:
      overhead_byte_count = 0;
      break;
    }

    if (update_crc) {
      ck_a += *in;
      ck_b += ck_a;
      update_crc = false;
    }

    in++;
  }

  if (overhead_byte_count > 0) {
    if (overhead_byte_count == UBLOX_PROTOCOL_OVERHEAD_LENGTH_BYTES) {
      *msg_length_bytes = msg_byte_count;
    }
  }

  if (rem != NULL) {
    *rem = in;
  }
}

void ublox_gnss_cfg_val_set(struct ublox_gnss_device *device, uint32_t key_id,
                            uint64_t value,
                            enum ublox_gnss_cfg_val_transaction transaction,
                            uint32_t layers) {}

void ublox_gnss_cfg_val_set_list(
    struct ublox_gnss_device *device, struct ublox_gnss_cfg_val *list,
    uint16_t number_of_values, enum ublox_gnss_cfg_val_transaction transaction,
    uint32_t layers) {
  uint16_t msg_length_bytes = 4 + (number_of_values << 2);

  for (uint16_t i = 0; i < number_of_values; i++) {
    msg_length_bytes += cfg_val_key_get_storage_size_bytes(
        UBLOX_GNSS_CFG_VAL_KEY_GET_SIZE((list + i)->key_id));
  }

  uint8_t msg[msg_length_bytes];

  *(msg) = 0x01;
  *(msg + 1) = (uint8_t)layers;
  *(msg + 2) = (uint8_t)transaction;
  *(msg + 3) = 0;

  uint8_t *tmp = msg + 4;
  uint16_t storage_size_bytes;

  for (uint16_t i = 0; i < number_of_values; i++) {
    // *((uint32_t *) tmp) = ublox_protocol_u32_encode(list->key_id);
    *((uint32_t *)tmp) = (list->key_id);
    tmp += sizeof(list->key_id);
    storage_size_bytes = cfg_val_key_get_storage_size_bytes(
        UBLOX_GNSS_CFG_VAL_KEY_GET_SIZE(list->key_id));

    switch (storage_size_bytes) {
    case 1:
      *(uint8_t *)tmp = *((const uint8_t *)&(list->value));
      break;
    case 2:
      *(uint16_t *)tmp =
          ublox_protocol_u16_encode(*(const uint16_t *)&(list->value));
      break;
    case 4:
      *(uint32_t *)tmp =
          ublox_protocol_u32_encode(*(const uint32_t *)&(list->value));
      break;
    case 8:
      *(uint64_t *)tmp =
          ublox_protocol_u64_encode(*(const uint64_t *)&(list->value));
      break;
    default:
      break;
    }

    tmp += storage_size_bytes;
    list++;
  }

  uint16_t ublox_msg_length_bytes =
      msg_length_bytes + UBLOX_PROTOCOL_OVERHEAD_LENGTH_BYTES;
  uint8_t tx[ublox_msg_length_bytes];
  ublox_protocol_encode(0x06, 0x8a, msg, msg_length_bytes, tx);
  ublox_gnss_send_msg(device, tx, ublox_msg_length_bytes);
}

void ublox_gnss_dec_ubx_nav_timeutc(
    uint8_t *msg, uint16_t msg_length_bytes,
    struct ublox_gnss_nav_timeutc *nav_timeutc) {
  if (msg_length_bytes != UBLOX_GNSS_DEC_UBX_NAV_TIMEUTC_BODY_LENGTH) {
    return;
  }

  struct ublox_gnss_nav_timeutc tmp;

  tmp.itow = ublox_protocol_u32_decode(msg);
  tmp.t_acc = ublox_protocol_u32_decode(msg + 4);
  tmp.nano = (int32_t)ublox_protocol_u32_decode(msg + 8);
  tmp.year = ublox_protocol_u16_decode(msg + 12);
  tmp.month = (uint8_t) * (msg + 14);
  tmp.day = (uint8_t) * (msg + 15);
  tmp.hour = (uint8_t) * (msg + 16);
  tmp.min = (uint8_t) * (msg + 17);
  tmp.sec = (uint8_t) * (msg + 18);
  tmp.valid = (uint8_t) * (msg + 19);

  memcpy(nav_timeutc, &tmp, sizeof(tmp));
}

void ublox_gnss_dec_ubx_nav_cov(uint8_t *msg, uint16_t msg_length_bytes,
                                struct ublox_gnss_nav_cov *nav_cov) {
  if (msg_length_bytes != UBLOX_GNSS_DEC_UBX_NAV_COV_BODY_LENGTH) {
    return;
  }

  struct ublox_gnss_nav_cov tmp;

  tmp.itow = ublox_protocol_u32_decode(msg);
  tmp.version = (uint8_t) * (msg + 4);
  tmp.pos_cov_valid = (uint8_t) * (msg + 5);
  tmp.vel_cov_valid = (uint8_t) * (msg + 6);
  tmp.pos_cov_nn = ublox_protocol_f32_decode(msg + 16);
  tmp.pos_cov_ne = ublox_protocol_f32_decode(msg + 20);
  tmp.pos_cov_nd = ublox_protocol_f32_decode(msg + 24);
  tmp.pos_cov_ee = ublox_protocol_f32_decode(msg + 28);
  tmp.pos_cov_ed = ublox_protocol_f32_decode(msg + 32);
  tmp.pos_cov_dd = ublox_protocol_f32_decode(msg + 36);
  tmp.vel_cov_nn = ublox_protocol_f32_decode(msg + 40);
  tmp.vel_cov_ne = ublox_protocol_f32_decode(msg + 44);
  tmp.vel_cov_nd = ublox_protocol_f32_decode(msg + 48);
  tmp.vel_cov_ee = ublox_protocol_f32_decode(msg + 52);
  tmp.vel_cov_ed = ublox_protocol_f32_decode(msg + 56);
  tmp.vel_cov_dd = ublox_protocol_f32_decode(msg + 60);

  memcpy(nav_cov, &tmp, sizeof(tmp));
}

void ublox_gnss_dec_ubx_nav_posecef(
    uint8_t *msg, uint16_t msg_length_bytes,
    struct ublox_gnss_nav_posecef *nav_posecef) {
  if (msg_length_bytes != UBLOX_GNSS_DEC_UBX_NAV_POSECEF_BODY_LENGTH) {
    return;
  }

  struct ublox_gnss_nav_posecef tmp;

  tmp.itow = ublox_protocol_u32_decode(msg);
  tmp.ecef_x = (int32_t)ublox_protocol_u32_decode(msg + 4);
  tmp.ecef_y = (int32_t)ublox_protocol_u32_decode(msg + 8);
  tmp.ecef_z = (int32_t)ublox_protocol_u32_decode(msg + 12);
  tmp.p_acc = ublox_protocol_u32_decode(msg + 16);

  memcpy(nav_posecef, &tmp, sizeof(tmp));
}
void ublox_gnss_dec_ubx_nav_hpposecef(uint8_t *msg, uint16_t msg_length_bytes,
                                     struct ublox_gnss_nav_hpposecef *nav_hpposecef) {
    if (msg_length_bytes != UBLOX_GNSS_DEC_UBX_NAV_HPPOSECEF_BODY_LENGTH) {
        return;
    }

    struct ublox_gnss_nav_hpposecef tmp;

    tmp.version = (uint32_t)*(msg);
    tmp.itow = ublox_protocol_u32_decode(msg + 4);
    tmp.ecefX = (int32_t)ublox_protocol_u32_decode(msg + 8);
    tmp.ecefY = (int32_t)ublox_protocol_u32_decode(msg + 12);
    tmp.ecefZ = (int32_t)ublox_protocol_u32_decode(msg + 16);
    tmp.ecefXHp = (int8_t)*(msg + 20);
    tmp.ecefYHp = (int8_t)*(msg + 21);
    tmp.ecefZHp = (int8_t)*(msg + 22);
    tmp.reserved1 = (uint8_t)*(msg + 23);
    tmp.pAcc = ublox_protocol_u32_decode(msg + 24);

    memcpy(nav_hpposecef, &tmp, sizeof(tmp));
}


void ublox_gnss_dec_ubx_nav_pvt(uint8_t *msg, uint16_t msg_length_bytes,
                                struct ublox_gnss_nav_pvt *nav_pvt) {
  if (msg_length_bytes != UBLOX_GNSS_DEC_UBX_NAV_PVT_BODY_LENGTH) {
    return;
  }

  struct ublox_gnss_nav_pvt tmp;

  tmp.itow = ublox_protocol_u32_decode(msg);
  tmp.year = ublox_protocol_u16_decode(msg + 4);
  tmp.month = (uint8_t) *(msg + 6);
  tmp.day = (uint8_t) *(msg + 7);
  tmp.hour = (uint8_t) *(msg + 8);
  tmp.min = (uint8_t) *(msg + 9);
  tmp.sec = (uint8_t) *(msg + 10);
  tmp.valid = (uint8_t) *(msg + 11);
  tmp.t_acc = ublox_protocol_u32_decode(msg + 12);
  tmp.nano = (int32_t) ublox_protocol_u32_decode(msg + 16);
  tmp.fix_type = (uint8_t) *(msg + 20);
  tmp.flags = (uint8_t) *(msg + 21);
  tmp.flags2 = (uint8_t) *(msg + 22);
  tmp.num_sv = (uint8_t) *(msg + 23);
  tmp.lon = (int32_t) ublox_protocol_u32_decode(msg + 24);
  tmp.lat = (int32_t) ublox_protocol_u32_decode(msg + 28);
  tmp.height = (int32_t) ublox_protocol_u32_decode(msg + 32);
  tmp.h_msl = (int32_t) ublox_protocol_u32_decode(msg + 36);
  tmp.h_acc = ublox_protocol_u32_decode(msg + 40);
  tmp.v_acc = ublox_protocol_u32_decode(msg + 44);
  tmp.vel_n = (int32_t) ublox_protocol_u32_decode(msg + 48);
  tmp.vel_e = (int32_t) ublox_protocol_u32_decode(msg + 52);
  tmp.vel_d = (int32_t) ublox_protocol_u32_decode(msg + 56);
  tmp.g_speed = (int32_t) ublox_protocol_u32_decode(msg + 60);
  tmp.head_mot = (int32_t) ublox_protocol_u32_decode(msg + 64);
  tmp.s_acc = ublox_protocol_u32_decode(msg + 68);
  tmp.head_acc = ublox_protocol_u32_decode(msg + 72);
  tmp.p_dop = ublox_protocol_u16_decode(msg + 76);
  tmp.flags3 = ublox_protocol_u16_decode(msg + 78);
  tmp.head_veh = (int32_t) ublox_protocol_u32_decode(msg + 84);
  tmp.mag_dec = (int16_t) ublox_protocol_u16_decode(msg + 88);
  tmp.mag_acc = ublox_protocol_u16_decode(msg + 90);
 
  memcpy(nav_pvt, &tmp, sizeof(tmp));

  return;
}

void ublox_gnss_dec_ubx_nav_posllh(uint8_t *msg, uint16_t msg_length_bytes,
                                   struct ublox_gnss_nav_posllh *nav_posllh) {
  if (msg_length_bytes != UBLOX_GNSS_DEC_UBX_NAV_POSLLH_BODY_LENGTH) {
    return;
  }

  struct ublox_gnss_nav_posllh tmp;

  tmp.itow = ublox_protocol_u32_decode(msg);
  tmp.lon = (int32_t)ublox_protocol_u32_decode(msg + 4);
  tmp.lat = (int32_t)ublox_protocol_u32_decode(msg + 8);
  tmp.height = (int32_t)ublox_protocol_u32_decode(msg + 12);
  tmp.h_msl = (int32_t)ublox_protocol_u32_decode(msg + 16);
  tmp.h_acc = ublox_protocol_u32_decode(msg + 20);
  tmp.v_acc = ublox_protocol_u32_decode(msg + 24);

  memcpy(nav_posllh, &tmp, sizeof(tmp));
}

void ublox_gnss_dec_ubx_nav_hppvt(uint8_t *msg, uint16_t msg_length_bytes,
                                 struct ublox_gnss_nav_hppvt *nav_hppvt) {
    if (msg_length_bytes != UBLOX_GNSS_DEC_UBX_NAV_HPPVT_BODY_LENGTH) {
        return;
    }

    struct ublox_gnss_nav_hppvt tmp;

    tmp.itow = ublox_protocol_u32_decode(msg);
    tmp.year = ublox_protocol_u16_decode(msg + 4);
    tmp.month = (uint8_t)*(msg + 6);
    tmp.day = (uint8_t)*(msg + 7);
    tmp.hour = (uint8_t)*(msg + 8);
    tmp.min = (uint8_t)*(msg + 9);
    tmp.sec = (uint8_t)*(msg + 10);
    tmp.valid = (uint8_t)*(msg + 11);
    tmp.tAcc = ublox_protocol_u32_decode(msg + 12);
    tmp.nano = (int32_t)ublox_protocol_u32_decode(msg + 16);
    tmp.fixType = (uint8_t)*(msg + 20);
    tmp.flags = (uint8_t)*(msg + 21);
    tmp.flags2 = (uint8_t)*(msg + 22);
    tmp.numSV = (uint8_t)*(msg + 23);
    tmp.lon = (int32_t)ublox_protocol_u32_decode(msg + 24);
    tmp.lat = (int32_t)ublox_protocol_u32_decode(msg + 28);
    tmp.height = (int32_t)ublox_protocol_u32_decode(msg + 32);
    tmp.hMSL = (int32_t)ublox_protocol_u32_decode(msg + 36);
    tmp.lonHp = (int8_t)*(msg + 40);
    tmp.latHp = (int8_t)*(msg + 41);
    tmp.heightHp = (int8_t)*(msg + 42);
    tmp.hMSLHp = (int8_t)*(msg + 43);
    tmp.hAcc = ublox_protocol_u32_decode(msg + 44);
    tmp.vAcc = ublox_protocol_u32_decode(msg + 48);
    tmp.velN = (int32_t)ublox_protocol_u32_decode(msg + 52);
    tmp.velE = (int32_t)ublox_protocol_u32_decode(msg + 56);
    tmp.velD = (int32_t)ublox_protocol_u32_decode(msg + 60);
    tmp.gSpeed = (int32_t)ublox_protocol_u32_decode(msg + 64);
    tmp.headMot = (int32_t)ublox_protocol_u32_decode(msg + 68);
    tmp.sAcc = ublox_protocol_u32_decode(msg + 72);
    tmp.headAcc = ublox_protocol_u32_decode(msg + 76);
    tmp.pDOP = ublox_protocol_u16_decode(msg + 80);
    tmp.flags3 = (uint8_t)*(msg + 82);
    tmp.headVeh = (int32_t)ublox_protocol_u32_decode(msg + 88);
    tmp.magDec = (int16_t)ublox_protocol_u16_decode(msg + 92);
    tmp.magAcc = ublox_protocol_u16_decode(msg + 94);

    memcpy(nav_hppvt, &tmp, sizeof(tmp));
}

__attribute__((weak)) enum ublox_gnss_err
ublox_gnss_send_msg(struct ublox_gnss_device *device, const uint8_t *buffer,
                    uint16_t size) {
  return UBLOX_GNSS_ERR_OK;
}
