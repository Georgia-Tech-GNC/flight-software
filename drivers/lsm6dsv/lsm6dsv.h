#ifndef LSM6DSV_H
#define LSM6DSV_H

#include "main.h"

#ifdef HAL_SPI_MODULE_ENABLED

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define LSM6DSV_WHO_AM_I_VALUE 0x70U

/*high performance mode*/
typedef enum {
    LSM6DSV_ODR_OFF = 0x0,
    LSM6DSV_ODR_7_5_HZ = 0x2,
    LSM6DSV_ODR_15_HZ = 0x3,
    LSM6DSV_ODR_30_HZ = 0x4,
    LSM6DSV_ODR_60_HZ = 0x5,
    LSM6DSV_ODR_120_HZ = 0x6,
    LSM6DSV_ODR_240_HZ = 0x7,
    LSM6DSV_ODR_480_HZ = 0x8,
    LSM6DSV_ODR_960_HZ = 0x9,
    LSM6DSV_ODR_1920_HZ = 0xA,
    LSM6DSV_ODR_3840_HZ = 0xB,
    LSM6DSV_ODR_7680_HZ = 0xC,
} lsm6dsv_odr_t;

/**Accelerometer full scale*/
typedef enum {
    LSM6DSV_ACCEL_FS_2G = 0x0,
    LSM6DSV_ACCEL_FS_4G = 0x1,
    LSM6DSV_ACCEL_FS_8G = 0x2,
    LSM6DSV_ACCEL_FS_16G = 0x3,
} lsm6dsv_accel_fs_t;

/** Gyroscope full scale*/
typedef enum {
    LSM6DSV_GYRO_FS_125DPS = 0x0,
    LSM6DSV_GYRO_FS_250DPS = 0x1,
    LSM6DSV_GYRO_FS_500DPS = 0x2,
    LSM6DSV_GYRO_FS_1000DPS = 0x3,
    LSM6DSV_GYRO_FS_2000DPS = 0x4,
    LSM6DSV_GYRO_FS_4000DPS = 0xC,
} lsm6dsv_gyro_fs_t;

/*Output data rate of the sensor fusion block*/
typedef enum {
    LSM6DSV_SFLP_ODR_15_HZ = 0x0,
    LSM6DSV_SFLP_ODR_30_HZ = 0x1,
    LSM6DSV_SFLP_ODR_60_HZ = 0x2,
    LSM6DSV_SFLP_ODR_120_HZ = 0x3,
    LSM6DSV_SFLP_ODR_240_HZ = 0x4,
    LSM6DSV_SFLP_ODR_480_HZ = 0x5,
} lsm6dsv_sflp_odr_t;

/*sensor config*/
typedef struct {
    lsm6dsv_odr_t accel_odr;
    lsm6dsv_accel_fs_t accel_fs;
    lsm6dsv_odr_t gyro_odr;
    lsm6dsv_gyro_fs_t gyro_fs;

    /*Run the on-chip sensor fusion and batch its outputs into FIFO */
    bool sflp_enable;
    /*Fusion rate*/
    lsm6dsv_sflp_odr_t sflp_odr;
    /*batch the gravity vector */
    bool sflp_gravity;
    /*batch the gyroscope bias estimate*/
    bool sflp_gyro_bias;
} lsm6dsv_config_t;

#define LSM6DSV_DEFAULT_CONFIG                                                                                         \
    ((lsm6dsv_config_t) {                                                                                              \
        .accel_odr = LSM6DSV_ODR_960_HZ,                                                                               \
        .accel_fs = LSM6DSV_ACCEL_FS_16G,                                                                              \
        .gyro_odr = LSM6DSV_ODR_960_HZ,                                                                                \
        .gyro_fs = LSM6DSV_GYRO_FS_2000DPS,                                                                            \
        .sflp_enable = true,                                                                                           \
        .sflp_odr = LSM6DSV_SFLP_ODR_480_HZ,                                                                           \
        .sflp_gravity = true,                                                                                          \
        .sflp_gyro_bias = true,                                                                                        \
    })

/*Device handle */
typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;

    lsm6dsv_config_t config;
    float accel_mps2_per_lsb;
    float gyro_rads_per_lsb;
    float timestamp_s_per_tick;
} lsm6dsv_t;

/*One reading of the output registers*/
typedef struct {
    float accel_mps2[3];
    float gyro_rads[3];
    float temperature_c;
} lsm6dsv_imu_data_t;

/*Bits of lsm6dsv_fusion_t*/
#define LSM6DSV_FUSION_QUAT (1U << 0)
#define LSM6DSV_FUSION_GRAVITY (1U << 1)
#define LSM6DSV_FUSION_GYRO_BIAS (1U << 2)

/*Latest sensor fusion output */
typedef struct {
    /**
     * Attitude quaternion {w, x, y, z}, rotating sensor-frame vectors into a world frame whose Z axis
     * points up and whose heading is wherever the sensor pointed when fusion started. w is always >= 0.
     */
    float quat[4];
    /*Gravity in the sensor frame*/
    float gravity_mps2[3];
    /*Gyroscope bias the filter is currently removing */
    float gyro_bias_rads[3];
    /*whic fields the last lsm6dsv_read_fusion() call refreshed */
    uint8_t updated;
    /*says FIFO overflowed since the last read, so some fusion samples lost */
    bool overrun;
} lsm6dsv_fusion_t;

/*results of lsm6dsv_self_test()*/
typedef struct {
    bool accel_pass;
    bool gyro_pass;
    float accel_delta_mg[3];
    float gyro_delta_dps[3];
} lsm6dsv_self_test_t;

/*Resets and configures the sensor*/
bool lsm6dsv_init(lsm6dsv_t *dev, const lsm6dsv_config_t *config);

/*reads the latest accelerometer, gyroscope, temperature outputs*/
bool lsm6dsv_read_imu(const lsm6dsv_t *dev, lsm6dsv_imu_data_t *out);

/*drains the FIFO and stores the newest sensor fusion outputs*/
bool lsm6dsv_read_fusion(const lsm6dsv_t *dev, lsm6dsv_fusion_t *fusion);

/*Restarts the sensor fusion filter*/
bool lsm6dsv_reset_fusion(const lsm6dsv_t *dev);

/*reads the free running timestamp counter*/
bool lsm6dsv_read_timestamp(const lsm6dsv_t *dev, uint32_t *ticks);

/**
 * Runs the built-in accelerometer and gyroscope self tests against the datasheet limits
 * The sensor must be still and The device must already be initialized
 */
bool lsm6dsv_self_test(lsm6dsv_t *dev, lsm6dsv_self_test_t *result);

/** Reads consecutive registers*/
bool lsm6dsv_read_regs(const lsm6dsv_t *dev, uint8_t reg, uint8_t *data, size_t len);

/*Writes one register */
bool lsm6dsv_write_reg(const lsm6dsv_t *dev, uint8_t reg, uint8_t value);

#endif

#endif
