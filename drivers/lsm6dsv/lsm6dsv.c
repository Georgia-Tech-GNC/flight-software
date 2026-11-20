#include "lsm6dsv.h"
#include "port.h"

#ifdef HAL_I2C_MODULE_ENABLED

#include "FreeRTOS.h"
#include "task.h"

#include <math.h>
#include <string.h>

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

#define FIFO_TAG_SHIFT 3U
#define FIFO_TAG_GYRO_NC 0x01U
#define FIFO_TAG_ACCEL_NC 0x02U
#define FIFO_CTRL3_BDR_GY_SHIFT 4U
#define ODR_7_5_HZ 7.5f
#define FIFO_WORD_BYTES 7U
#define FIFO_MAX_WORDS 511U

#define I2C_TIMEOUT_MS 10U
#define I2C_BUSY_RETRIES 10U
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

/*bus access*/
static uint16_t bus_address(const lsm6dsv_t *dev) {
    UNUSED(dev);
    return (uint16_t) ((uint16_t) lsm6dsv_i2c_addr << 1U);
}

bool lsm6dsv_read_regs(const lsm6dsv_t *dev, uint8_t reg, uint8_t *data, size_t len) {
    if (len == 0U || len > MAX_BURST) {
        return false;
    }
    HAL_StatusTypeDef status;
    for (uint32_t attempt = 0; attempt <= I2C_BUSY_RETRIES; attempt++) {
        WITH_I2C(lsm6dsv, 
            status = HAL_I2C_Mem_Read(i2c, bus_address(dev), reg, I2C_MEMADD_SIZE_8BIT, data,
                                                    (uint16_t) len, I2C_TIMEOUT_MS);
        );
        if (status != HAL_BUSY) {
            return status == HAL_OK;
        }
        delay_ms(1);
    }
    return false;
}

bool lsm6dsv_write_reg(const lsm6dsv_t *dev, uint8_t reg, uint8_t value) {
    UNUSED(dev);
    HAL_StatusTypeDef status;
    for (uint32_t attempt = 0; attempt <= I2C_BUSY_RETRIES; attempt++) {
        WITH_I2C(lsm6dsv, 
            status = HAL_I2C_Mem_Write(i2c, bus_address(dev), reg, I2C_MEMADD_SIZE_8BIT, &value, 1U, I2C_TIMEOUT_MS);
        );
        if (status != HAL_BUSY) {
            return status == HAL_OK;
        }
        delay_ms(1);
    }
    return false;
}

static bool update_bits(const lsm6dsv_t *dev, uint8_t reg, uint8_t mask, uint8_t value) {
    uint8_t current = 0;
    if (!lsm6dsv_read_regs(dev, reg, &current, 1)) {
        return false;
    }
    return lsm6dsv_write_reg(dev, reg, (uint8_t) ((current & (uint8_t) ~mask) | (value & mask)));
}

/*Config*/
static bool config_is_valid(const lsm6dsv_config_t *config) {
    bool gyro_fs_ok = config->gyro_fs <= LSM6DSV_GYRO_FS_2000DPS || config->gyro_fs == LSM6DSV_GYRO_FS_4000DPS;
    bool odr_ok = config->accel_odr <= LSM6DSV_ODR_7680_HZ && config->gyro_odr <= LSM6DSV_ODR_7680_HZ;
    bool fs_ok = config->accel_fs <= LSM6DSV_ACCEL_FS_16G && gyro_fs_ok;

    return odr_ok && fs_ok;
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

static bool stop_raw_batching(const lsm6dsv_t *dev) {
    return lsm6dsv_write_reg(dev, REG_FIFO_CTRL4, FIFO_MODE_BYPASS);
}

static bool start_raw_batching(const lsm6dsv_t *dev) {
    const lsm6dsv_config_t *config = &dev->config;
    uint8_t bdr =
        (uint8_t) (((unsigned int) config->gyro_odr << FIFO_CTRL3_BDR_GY_SHIFT) | (unsigned int) config->accel_odr);
    return lsm6dsv_write_reg(dev, REG_FIFO_CTRL3, bdr) && lsm6dsv_write_reg(dev, REG_FIFO_CTRL4, FIFO_MODE_CONTINUOUS);
}

static float sample_period_s(const lsm6dsv_t *dev, lsm6dsv_odr_t odr) {
    if (odr < LSM6DSV_ODR_7_5_HZ) {
        return 0.0f;
    }
    float nominal_hz = ODR_7_5_HZ * (float) (1U << ((unsigned int) odr - (unsigned int) LSM6DSV_ODR_7_5_HZ));
    return TIMESTAMP_BASE_HZ * dev->timestamp_s_per_tick / nominal_hz;
}

static bool apply_config(lsm6dsv_t *dev) {
    const lsm6dsv_config_t *config = &dev->config;

    dev->accel_mps2_per_lsb = accel_mg_per_lsb(config->accel_fs) * 0.001f * STANDARD_GRAVITY;
    dev->gyro_rads_per_lsb = gyro_mdps_per_lsb(config->gyro_fs) * 0.001f * DEG_TO_RAD;
    dev->accel_sample_period_s = sample_period_s(dev, config->accel_odr);
    dev->gyro_sample_period_s = sample_period_s(dev, config->gyro_odr);

    bool ok = lsm6dsv_write_reg(dev, REG_CTRL3, CTRL3_BDU | CTRL3_IF_INC)
              && update_bits(dev, REG_CTRL8, CTRL8_FS_XL_MASK, (uint8_t) config->accel_fs)
              && update_bits(dev, REG_CTRL6, CTRL6_FS_G_MASK, (uint8_t) config->gyro_fs)
              && lsm6dsv_write_reg(dev, REG_CTRL10, 0)
              && update_bits(dev, REG_FUNCTIONS_ENABLE, FUNCTIONS_ENABLE_TIMESTAMP_EN, FUNCTIONS_ENABLE_TIMESTAMP_EN)
              /*high-performance mode */
              && lsm6dsv_write_reg(dev, REG_CTRL1, (uint8_t) config->accel_odr)
              && lsm6dsv_write_reg(dev, REG_CTRL2, (uint8_t) config->gyro_odr);

    return ok && (!config->batch_raw || start_raw_batching(dev));
}

bool lsm6dsv_init(lsm6dsv_t *dev, const lsm6dsv_config_t *config) {
    if (!config_is_valid(config)) {
        return false;
    }
    dev->config = *config;

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

static void store_raw_sample(float samples[][3], uint16_t *count, const uint8_t *data, float scale, bool *dropped) {
    if (*count >= LSM6DSV_MAX_RAW_SAMPLES) {
        *dropped = true;
        return;
    }
    for (size_t axis = 0; axis < 3U; axis++) {
        samples[*count][axis] = (float) le_i16(&data[2U * axis]) * scale;
    }
    *count = (uint16_t) (*count + 1U);
}

static void decode_fifo_word(const lsm6dsv_t *dev, const uint8_t word[FIFO_WORD_BYTES], lsm6dsv_raw_samples_t *raw) {
    const uint8_t *data = &word[1];

    switch (word[0] >> FIFO_TAG_SHIFT) {
    case FIFO_TAG_GYRO_NC:
        store_raw_sample(raw->gyro_rads, &raw->gyro_count, data, dev->gyro_rads_per_lsb, &raw->dropped);
        break;
    case FIFO_TAG_ACCEL_NC:
        store_raw_sample(raw->accel_mps2, &raw->accel_count, data, dev->accel_mps2_per_lsb, &raw->dropped);
        break;
    default:
        break;
    }
}

bool lsm6dsv_read_fifo(const lsm6dsv_t *dev, lsm6dsv_raw_samples_t *raw) {
    uint8_t status[2] = {0};

    raw->gyro_count = 0;
    raw->accel_count = 0;
    raw->dropped = false;
    raw->overrun = false;

    if (!lsm6dsv_read_regs(dev, REG_FIFO_STATUS1, status, sizeof(status))) {
        return false;
    }
    uint16_t count = (uint16_t) ((uint16_t) status[0] | (uint16_t) ((status[1] & FIFO_STATUS2_DIFF_FIFO_8) << 8U));
    raw->overrun = (status[1] & (FIFO_STATUS2_OVR_IA | FIFO_STATUS2_OVR_LATCHED)) != 0U;

    if (count > FIFO_MAX_WORDS) {
        count = FIFO_MAX_WORDS;
    }

    for (uint16_t i = 0; i < count; i++) {
        uint8_t word[FIFO_WORD_BYTES] = {0};
        if (!lsm6dsv_read_regs(dev, REG_FIFO_DATA_OUT_TAG, word, sizeof(word))) {
            return false;
        }
        decode_fifo_word(dev, word, raw);
    }
    return true;
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

    bool ok = stop_raw_batching(dev) && run_self_test(dev, &ACCEL_SELF_TEST, accel_delta)
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
