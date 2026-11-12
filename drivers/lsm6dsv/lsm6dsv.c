#include "lsm6dsv.h"

#ifdef HAL_SPI_MODULE_ENABLED

#include "FreeRTOS.h"
#include "task.h"

#include <math.h>
#include <string.h>

#define REG_FUNC_CFG_ACCESS 0x01U
#define REG_IF_CFG 0x03U
#define REG_FIFO_CTRL3 0x09U
#define REG_FIFO_CTRL4 0x0AU
#define REG_WHO_AM_I 0x0FU
#define REG_CTRL1 0x10U
#define REG_CTRL2 0x11U
#define REG_CTRL3 0x12U
#define REG_CTRL6 0x15U
#define REG_CTRL8 0x17U
#define REG_CTRL10 0x19U
#define REG_FIFO_STATUS1 0x1BU
#define REG_STATUS 0x1EU
#define REG_OUT_TEMP_L 0x20U
#define REG_OUTX_L_G 0x22U
#define REG_OUTX_L_A 0x28U
#define REG_TIMESTAMP0 0x40U
#define REG_INTERNAL_FREQ_FINE 0x4FU
#define REG_FUNCTIONS_ENABLE 0x50U
#define REG_FIFO_DATA_OUT_TAG 0x78U

#define EMB_FUNC_EN_A 0x04U
#define EMB_FUNC_FIFO_EN_A 0x44U
#define EMB_SFLP_ODR 0x5EU
#define EMB_FUNC_INIT_A 0x66U

#define FUNC_CFG_ACCESS_EMB_FUNC_REG_ACCESS 0x80U
#define IF_CFG_I2C_I3C_DISABLE 0x01U
#define CTRL3_BDU 0x40U
#define CTRL3_IF_INC 0x04U
#define CTRL3_SW_RESET 0x01U
#define CTRL6_FS_G_MASK 0x0FU
#define CTRL8_FS_XL_MASK 0x03U
#define CTRL10_ST_XL_POSITIVE 0x01U
#define CTRL10_ST_G_POSITIVE 0x04U
#define STATUS_XLDA 0x01U
#define STATUS_GDA 0x02U
#define FIFO_STATUS2_OVR_IA 0x40U
#define FIFO_STATUS2_OVR_LATCHED 0x08U
#define FIFO_STATUS2_DIFF_FIFO_8 0x01U
#define FIFO_MODE_BYPASS 0x00U
#define FIFO_MODE_CONTINUOUS 0x06U
#define FUNCTIONS_ENABLE_TIMESTAMP_EN 0x40U

#define EMB_FUNC_EN_A_SFLP_GAME_EN 0x02U
#define EMB_FUNC_FIFO_EN_A_SFLP_GAME 0x02U
#define EMB_FUNC_FIFO_EN_A_SFLP_GRAVITY 0x10U
#define EMB_FUNC_FIFO_EN_A_SFLP_GBIAS 0x20U
#define EMB_SFLP_ODR_SHIFT 3U
#define EMB_SFLP_ODR_MASK 0x38U
#define EMB_FUNC_INIT_A_SFLP_GAME_INIT 0x02U

/* FIFO_DATA_OUT_TAG: TAG_SENSOR in bits 7:3*/
#define FIFO_TAG_SHIFT 3U
#define FIFO_TAG_SFLP_GAME_ROTATION 0x13U
#define FIFO_TAG_SFLP_GYRO_BIAS 0x16U
#define FIFO_TAG_SFLP_GRAVITY 0x17U
#define FIFO_WORD_BYTES 7U
#define FIFO_MAX_WORDS 511U /*DIFF_FIFO is 9 bits*/

#define SPI_TIMEOUT_MS 10U
#define MAX_BURST 16U
#define BOOT_TIME_MS 10U
#define RESET_TIMEOUT_MS 10U
#define DRDY_TIMEOUT_MS 50U
#define SELF_TEST_SETTLE_MS 100U
#define SELF_TEST_SAMPLES 5

/*Self-test pass windows*/
#define ST_ACCEL_MIN_MG 50.0f
#define ST_ACCEL_MAX_MG 1700.0f
#define ST_GYRO_MIN_DPS 150.0f /*at +/-2000 dps*/
#define ST_GYRO_MAX_DPS 700.0f

#define STANDARD_GRAVITY 9.80665f
#define DEG_TO_RAD 0.017453292519943295f
#define ACCEL_2G_MG_PER_LSB 0.061f
#define GYRO_125DPS_MDPS_PER_LSB 4.375f
#define ACCEL_4G_MG_PER_LSB 0.122f
#define GYRO_2000DPS_DPS_PER_LSB 0.070f
#define TEMP_LSB_PER_C 256.0f
#define TEMP_OFFSET_C 25.0f
#define TIMESTAMP_BASE_HZ 46080.0f
#define FREQ_FINE_STEP 0.0013f
/*SFLP payloads use fixed scales*/
#define SFLP_GRAVITY_MPS2_PER_LSB (ACCEL_2G_MG_PER_LSB * 0.001f * STANDARD_GRAVITY)
#define SFLP_GBIAS_RADS_PER_LSB (GYRO_125DPS_MDPS_PER_LSB * 0.001f * DEG_TO_RAD)

static void delay_ms(uint32_t ms) {
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        vTaskDelay(pdMS_TO_TICKS(ms));
    } else {
        HAL_Delay(ms);
    }
}

static int16_t le_i16(const uint8_t *bytes) {
    return (int16_t) (uint16_t) ((uint16_t) bytes[0] | (uint16_t) ((uint16_t) bytes[1] << 8U));
}

static uint16_t le_u16(const uint8_t *bytes) {
    return (uint16_t) ((uint16_t) bytes[0] | (uint16_t) ((uint16_t) bytes[1] << 8U));
}

/** IEEE 754 half precision (as used by the SFLP quaternion) to single precision */
static float half_to_float(uint16_t half) {
    uint32_t sign = ((uint32_t) half & 0x8000U) << 16U;
    uint32_t exponent = ((uint32_t) half >> 10U) & 0x1FU;
    uint32_t mantissa = (uint32_t) half & 0x3FFU;
    uint32_t bits;

    if (exponent == 0U) {
        if (mantissa == 0U) {
            bits = sign;
        } else {
            /*Renormalize into a normal float */
            exponent = 127U - 15U + 1U;
            while ((mantissa & 0x400U) == 0U) {
                mantissa <<= 1U;
                exponent--;
            }
            bits = sign | (exponent << 23U) | ((mantissa & 0x3FFU) << 13U);
        }
    } else if (exponent == 0x1FU) {
        bits = sign | 0x7F800000U | (mantissa << 13U); /* inf / NaN */
    } else {
        bits = sign | ((exponent + 127U - 15U) << 23U) | (mantissa << 13U);
    }

    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

/*bus access*/
static bool transfer(const lsm6dsv_t *dev, const uint8_t *tx, uint8_t *rx, uint16_t len) {
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(dev->hspi, tx, rx, len, SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
    return status == HAL_OK;
}

bool lsm6dsv_read_regs(const lsm6dsv_t *dev, uint8_t reg, uint8_t *data, size_t len) {
    uint8_t tx[MAX_BURST + 1U] = {0};
    uint8_t rx[MAX_BURST + 1U] = {0};

    if (len == 0U || len > MAX_BURST) {
        return false;
    }

    tx[0] = (uint8_t) (reg | 0x80U); /* bit 7 set = read */
    if (!transfer(dev, tx, rx, (uint16_t) (len + 1U))) {
        return false;
    }
    memcpy(data, &rx[1], len);
    return true;
}

bool lsm6dsv_write_reg(const lsm6dsv_t *dev, uint8_t reg, uint8_t value) {
    const uint8_t tx[2] = {(uint8_t) (reg & 0x7FU), value};
    uint8_t rx[2] = {0};
    return transfer(dev, tx, rx, sizeof(tx));
}

static bool update_bits(const lsm6dsv_t *dev, uint8_t reg, uint8_t mask, uint8_t value) {
    uint8_t current = 0;
    if (!lsm6dsv_read_regs(dev, reg, &current, 1)) {
        return false;
    }
    return lsm6dsv_write_reg(dev, reg, (uint8_t) ((current & (uint8_t) ~mask) | (value & mask)));
}

static bool set_embedded_bank(const lsm6dsv_t *dev, bool enable) {
    return update_bits(dev, REG_FUNC_CFG_ACCESS, FUNC_CFG_ACCESS_EMB_FUNC_REG_ACCESS,
                       enable ? FUNC_CFG_ACCESS_EMB_FUNC_REG_ACCESS : 0U);
}

/*Config*/
static bool config_is_valid(const lsm6dsv_config_t *config) {
    bool gyro_fs_ok = config->gyro_fs <= LSM6DSV_GYRO_FS_2000DPS || config->gyro_fs == LSM6DSV_GYRO_FS_4000DPS;
    bool odr_ok = config->accel_odr <= LSM6DSV_ODR_7680_HZ && config->gyro_odr <= LSM6DSV_ODR_7680_HZ;
    bool fs_ok = config->accel_fs <= LSM6DSV_ACCEL_FS_16G && gyro_fs_ok;

    if (!odr_ok || !fs_ok) {
        return false;
    }
    if (!config->sflp_enable) {
        return true;
    }

    /*SFLP_ODR codes 0..5 (15..480 Hz) line up with ODR codes 3..8*/
    unsigned int min_odr = (unsigned int) config->sflp_odr + (unsigned int) LSM6DSV_ODR_15_HZ;
    return config->sflp_odr <= LSM6DSV_SFLP_ODR_480_HZ && (unsigned int) config->accel_odr >= min_odr
           && (unsigned int) config->gyro_odr >= min_odr;
}

static float accel_mg_per_lsb(lsm6dsv_accel_fs_t fs) {
    return ACCEL_2G_MG_PER_LSB * (float) (1U << (unsigned int) fs);
}

static float gyro_mdps_per_lsb(lsm6dsv_gyro_fs_t fs) {
    /*125 dps doubles at each step up to 2000 dps*/
    unsigned int steps = (fs == LSM6DSV_GYRO_FS_4000DPS) ? 5U : (unsigned int) fs;
    return GYRO_125DPS_MDPS_PER_LSB * (float) (1U << steps);
}

static bool software_reset(const lsm6dsv_t *dev) {
    /*Powers both sensors down before resetting the control registers*/
    if (!lsm6dsv_write_reg(dev, REG_CTRL1, 0) || !lsm6dsv_write_reg(dev, REG_CTRL2, 0)
        || !lsm6dsv_write_reg(dev, REG_CTRL3, CTRL3_SW_RESET)) {
        return false;
    }

    for (uint32_t elapsed = 0; elapsed <= RESET_TIMEOUT_MS; elapsed++) {
        uint8_t ctrl3 = CTRL3_SW_RESET;
        if (!lsm6dsv_read_regs(dev, REG_CTRL3, &ctrl3, 1)) {
            return false;
        }
        if ((ctrl3 & CTRL3_SW_RESET) == 0U) {
            return true;
        }
        delay_ms(1);
    }
    return false;
}

/**Turns sensor fusion off and empties the FIFO*/
static bool stop_fusion(const lsm6dsv_t *dev) {
    if (!lsm6dsv_write_reg(dev, REG_FIFO_CTRL4, FIFO_MODE_BYPASS) || !set_embedded_bank(dev, true)) {
        return false;
    }
    bool ok = lsm6dsv_write_reg(dev, EMB_FUNC_EN_A, 0) && lsm6dsv_write_reg(dev, EMB_FUNC_FIFO_EN_A, 0);
    return set_embedded_bank(dev, false) && ok;
}

static bool start_fusion(const lsm6dsv_t *dev) {
    const lsm6dsv_config_t *config = &dev->config;
    uint8_t fifo_outputs = EMB_FUNC_FIFO_EN_A_SFLP_GAME;
    if (config->sflp_gravity) {
        fifo_outputs |= EMB_FUNC_FIFO_EN_A_SFLP_GRAVITY;
    }
    if (config->sflp_gyro_bias) {
        fifo_outputs |= EMB_FUNC_FIFO_EN_A_SFLP_GBIAS;
    }

    /*Bypass mode empties the FIFO; raw accel/gyro are read from the output registers instead*/
    if (!lsm6dsv_write_reg(dev, REG_FIFO_CTRL4, FIFO_MODE_BYPASS) || !lsm6dsv_write_reg(dev, REG_FIFO_CTRL3, 0)
        || !set_embedded_bank(dev, true)) {
        return false;
    }

    bool ok = lsm6dsv_write_reg(dev, EMB_FUNC_FIFO_EN_A, fifo_outputs)
              && update_bits(dev, EMB_SFLP_ODR, EMB_SFLP_ODR_MASK,
                             (uint8_t) ((unsigned int) config->sflp_odr << EMB_SFLP_ODR_SHIFT))
              && lsm6dsv_write_reg(dev, EMB_FUNC_EN_A, EMB_FUNC_EN_A_SFLP_GAME_EN)
              && lsm6dsv_write_reg(dev, EMB_FUNC_INIT_A, EMB_FUNC_INIT_A_SFLP_GAME_INIT);

    ok = set_embedded_bank(dev, false) && ok;
    /*Continuous mode. when full, the oldst entry is overwriten*/
    return ok && lsm6dsv_write_reg(dev, REG_FIFO_CTRL4, FIFO_MODE_CONTINUOUS);
}

static bool apply_config(lsm6dsv_t *dev) {
    const lsm6dsv_config_t *config = &dev->config;

    dev->accel_mps2_per_lsb = accel_mg_per_lsb(config->accel_fs) * 0.001f * STANDARD_GRAVITY;
    dev->gyro_rads_per_lsb = gyro_mdps_per_lsb(config->gyro_fs) * 0.001f * DEG_TO_RAD;

    bool ok = lsm6dsv_write_reg(dev, REG_CTRL3, CTRL3_BDU | CTRL3_IF_INC)
              /*SPI only*/
              && update_bits(dev, REG_IF_CFG, IF_CFG_I2C_I3C_DISABLE, IF_CFG_I2C_I3C_DISABLE)
              && update_bits(dev, REG_CTRL8, CTRL8_FS_XL_MASK, (uint8_t) config->accel_fs)
              && update_bits(dev, REG_CTRL6, CTRL6_FS_G_MASK, (uint8_t) config->gyro_fs)
              && lsm6dsv_write_reg(dev, REG_CTRL10, 0)
              && update_bits(dev, REG_FUNCTIONS_ENABLE, FUNCTIONS_ENABLE_TIMESTAMP_EN, FUNCTIONS_ENABLE_TIMESTAMP_EN)
              /*high-performance mode */
              && lsm6dsv_write_reg(dev, REG_CTRL1, (uint8_t) config->accel_odr)
              && lsm6dsv_write_reg(dev, REG_CTRL2, (uint8_t) config->gyro_odr);

    if (!ok) {
        return false;
    }
    return config->sflp_enable ? start_fusion(dev) : stop_fusion(dev);
}

bool lsm6dsv_init(lsm6dsv_t *dev, const lsm6dsv_config_t *config) {
    if (!config_is_valid(config)) {
        return false;
    }
    dev->config = *config;

    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
    delay_ms(BOOT_TIME_MS);

    uint8_t who_am_i = 0;
    if (!lsm6dsv_read_regs(dev, REG_WHO_AM_I, &who_am_i, 1) || who_am_i != LSM6DSV_WHO_AM_I_VALUE) {
        return false;
    }
    if (!software_reset(dev)) {
        return false;
    }

    /* Per-part trim of the internal oscillator, which clocks both the ODR and the timestamp */
    uint8_t freq_fine = 0;
    if (!lsm6dsv_read_regs(dev, REG_INTERNAL_FREQ_FINE, &freq_fine, 1)) {
        return false;
    }
    dev->timestamp_s_per_tick = 1.0f / (TIMESTAMP_BASE_HZ * (1.0f + FREQ_FINE_STEP * (float) (int8_t) freq_fine));

    return apply_config(dev);
}

/*data*/
bool lsm6dsv_read_imu(const lsm6dsv_t *dev, lsm6dsv_imu_data_t *out) {
    /*emperature, gyro XYZ, accel XYZ */
    uint8_t raw[14] = {0};
    if (!lsm6dsv_read_regs(dev, REG_OUT_TEMP_L, raw, sizeof(raw))) {
        return false;
    }

    out->temperature_c = (float) le_i16(&raw[0]) / TEMP_LSB_PER_C + TEMP_OFFSET_C;
    for (size_t axis = 0; axis < 3U; axis++) {
        out->gyro_rads[axis] = (float) le_i16(&raw[2U + 2U * axis]) * dev->gyro_rads_per_lsb;
        out->accel_mps2[axis] = (float) le_i16(&raw[8U + 2U * axis]) * dev->accel_mps2_per_lsb;
    }
    return true;
}

bool lsm6dsv_read_timestamp(const lsm6dsv_t *dev, uint32_t *ticks) {
    uint8_t raw[4] = {0};
    if (!lsm6dsv_read_regs(dev, REG_TIMESTAMP0, raw, sizeof(raw))) {
        return false;
    }
    *ticks = (uint32_t) raw[0] | ((uint32_t) raw[1] << 8U) | ((uint32_t) raw[2] << 16U) | ((uint32_t) raw[3] << 24U);
    return true;
}

static void decode_quaternion(const uint8_t *data, float quat[4]) {
    /*The FIFO carries x, y, z as half floats*/
    float x = half_to_float(le_u16(&data[0]));
    float y = half_to_float(le_u16(&data[2]));
    float z = half_to_float(le_u16(&data[4]));
    float sum_sq = x * x + y * y + z * z;

    if (sum_sq > 1.0f) {
        /*Half-float rounding can put xyz just past 1 */
        float norm = sqrtf(sum_sq);
        x /= norm;
        y /= norm;
        z /= norm;
        sum_sq = 1.0f;
    }

    quat[0] = sqrtf(1.0f - sum_sq);
    quat[1] = x;
    quat[2] = y;
    quat[3] = z;
}

static void decode_fifo_word(const uint8_t word[FIFO_WORD_BYTES], lsm6dsv_fusion_t *fusion) {
    const uint8_t *data = &word[1];

    switch (word[0] >> FIFO_TAG_SHIFT) {
    case FIFO_TAG_SFLP_GAME_ROTATION:
        decode_quaternion(data, fusion->quat);
        fusion->updated = (uint8_t) (fusion->updated | LSM6DSV_FUSION_QUAT);
        break;
    case FIFO_TAG_SFLP_GRAVITY:
        for (size_t axis = 0; axis < 3U; axis++) {
            fusion->gravity_mps2[axis] = (float) le_i16(&data[2U * axis]) * SFLP_GRAVITY_MPS2_PER_LSB;
        }
        fusion->updated = (uint8_t) (fusion->updated | LSM6DSV_FUSION_GRAVITY);
        break;
    case FIFO_TAG_SFLP_GYRO_BIAS:
        for (size_t axis = 0; axis < 3U; axis++) {
            fusion->gyro_bias_rads[axis] = (float) le_i16(&data[2U * axis]) * SFLP_GBIAS_RADS_PER_LSB;
        }
        fusion->updated = (uint8_t) (fusion->updated | LSM6DSV_FUSION_GYRO_BIAS);
        break;
    default:
        break;
    }
}

bool lsm6dsv_read_fusion(const lsm6dsv_t *dev, lsm6dsv_fusion_t *fusion) {
    uint8_t status[2] = {0};

    fusion->updated = 0;
    fusion->overrun = false;

    if (!lsm6dsv_read_regs(dev, REG_FIFO_STATUS1, status, sizeof(status))) {
        return false;
    }
    uint16_t count = (uint16_t) ((uint16_t) status[0] | (uint16_t) ((status[1] & FIFO_STATUS2_DIFF_FIFO_8) << 8U));
    fusion->overrun = (status[1] & (FIFO_STATUS2_OVR_IA | FIFO_STATUS2_OVR_LATCHED)) != 0U;

    if (count > FIFO_MAX_WORDS) {
        count = FIFO_MAX_WORDS;
    }

    for (uint16_t i = 0; i < count; i++) {
        uint8_t word[FIFO_WORD_BYTES] = {0};
        if (!lsm6dsv_read_regs(dev, REG_FIFO_DATA_OUT_TAG, word, sizeof(word))) {
            return false;
        }
        decode_fifo_word(word, fusion);
    }
    return true;
}

bool lsm6dsv_reset_fusion(const lsm6dsv_t *dev) {
    if (!dev->config.sflp_enable || !set_embedded_bank(dev, true)) {
        return false;
    }
    bool ok = lsm6dsv_write_reg(dev, EMB_FUNC_INIT_A, EMB_FUNC_INIT_A_SFLP_GAME_INIT);
    return set_embedded_bank(dev, false) && ok;
}

/*self test*/
static bool wait_data_ready(const lsm6dsv_t *dev, uint8_t mask) {
    for (uint32_t elapsed = 0; elapsed <= DRDY_TIMEOUT_MS; elapsed++) {
        uint8_t status = 0;
        if (!lsm6dsv_read_regs(dev, REG_STATUS, &status, 1)) {
            return false;
        }
        if ((status & mask) != 0U) {
            return true;
        }
        delay_ms(1);
    }
    return false;
}

/*averages SELF_TEST_SAMPLES readings of 3-axis output, getting rid of first one*/
static bool average_axes(const lsm6dsv_t *dev, uint8_t out_reg, uint8_t drdy_mask, float average_lsb[3]) {
    int32_t sum[3] = {0};

    for (int sample = 0; sample <= SELF_TEST_SAMPLES; sample++) {
        uint8_t raw[6] = {0};
        if (!wait_data_ready(dev, drdy_mask) || !lsm6dsv_read_regs(dev, out_reg, raw, sizeof(raw))) {
            return false;
        }
        if (sample == 0) {
            continue;
        }
        for (size_t axis = 0; axis < 3U; axis++) {
            sum[axis] += le_i16(&raw[2U * axis]);
        }
    }

    for (size_t axis = 0; axis < 3U; axis++) {
        average_lsb[axis] = (float) sum[axis] / (float) SELF_TEST_SAMPLES;
    }
    return true;
}

typedef struct {
    uint8_t fs_reg;
    uint8_t fs_mask;
    uint8_t fs_value;
    uint8_t odr_reg;
    uint8_t odr_value;
    uint8_t out_reg;
    uint8_t drdy_mask;
    uint8_t st_bits;
} self_test_setup_t;

/* Accelerometer */
static const self_test_setup_t ACCEL_SELF_TEST = {
    REG_CTRL8,    CTRL8_FS_XL_MASK, (uint8_t) LSM6DSV_ACCEL_FS_4G, REG_CTRL1, (uint8_t) LSM6DSV_ODR_60_HZ,
    REG_OUTX_L_A, STATUS_XLDA,      CTRL10_ST_XL_POSITIVE,
};

/*Gyroscope */
static const self_test_setup_t GYRO_SELF_TEST = {
    REG_CTRL6,    CTRL6_FS_G_MASK, (uint8_t) LSM6DSV_GYRO_FS_2000DPS, REG_CTRL2, (uint8_t) LSM6DSV_ODR_240_HZ,
    REG_OUTX_L_G, STATUS_GDA,      CTRL10_ST_G_POSITIVE,
};

/* Runs one sensor with and without the self test force*/
static bool run_self_test(const lsm6dsv_t *dev, const self_test_setup_t *setup, float delta[3]) {
    float off[3] = {0};
    float on[3] = {0};

    if (!lsm6dsv_write_reg(dev, REG_CTRL1, 0) || !lsm6dsv_write_reg(dev, REG_CTRL2, 0)
        || !update_bits(dev, setup->fs_reg, setup->fs_mask, setup->fs_value)
        || !lsm6dsv_write_reg(dev, setup->odr_reg, setup->odr_value)) {
        return false;
    }

    delay_ms(SELF_TEST_SETTLE_MS);
    if (!average_axes(dev, setup->out_reg, setup->drdy_mask, off)
        || !lsm6dsv_write_reg(dev, REG_CTRL10, setup->st_bits)) {
        return false;
    }

    delay_ms(SELF_TEST_SETTLE_MS);
    bool ok = average_axes(dev, setup->out_reg, setup->drdy_mask, on);
    /* Always try to switch the test force back off */
    ok = lsm6dsv_write_reg(dev, REG_CTRL10, 0) && ok;

    for (size_t axis = 0; axis < 3U; axis++) {
        delta[axis] = fabsf(on[axis] - off[axis]);
    }
    return ok;
}

static bool all_within(const float values[3], float min, float max) {
    for (size_t axis = 0; axis < 3U; axis++) {
        if (values[axis] < min || values[axis] > max) {
            return false;
        }
    }
    return true;
}

bool lsm6dsv_self_test(lsm6dsv_t *dev, lsm6dsv_self_test_t *result) {
    float accel_delta[3] = {0};
    float gyro_delta[3] = {0};

    memset(result, 0, sizeof(*result));

    bool ok = stop_fusion(dev) && run_self_test(dev, &ACCEL_SELF_TEST, accel_delta)
              && run_self_test(dev, &GYRO_SELF_TEST, gyro_delta);

    for (size_t axis = 0; axis < 3U; axis++) {
        result->accel_delta_mg[axis] = accel_delta[axis] * ACCEL_4G_MG_PER_LSB;
        result->gyro_delta_dps[axis] = gyro_delta[axis] * GYRO_2000DPS_DPS_PER_LSB;
    }
    result->accel_pass = ok && all_within(result->accel_delta_mg, ST_ACCEL_MIN_MG, ST_ACCEL_MAX_MG);
    result->gyro_pass = ok && all_within(result->gyro_delta_dps, ST_GYRO_MIN_DPS, ST_GYRO_MAX_DPS);

    return apply_config(dev) && ok;
}

#endif
