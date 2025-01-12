#include "sensors.h"
#include "gen_constants.h"

I2C_HandleTypeDef hi2c4;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi4;
SPI_HandleTypeDef hspi6;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

PCD_HandleTypeDef hpcd_USB_OTG_HS;

struct ADIS_Device imu_device;
struct lis3mdl_device mag_device;
MS5607StateTypeDef ms5607_state;

struct ublox_gnss_device gps;
__attribute__((section(".buffer"))) uint8_t uart4_rx_dma_buffer[1024];
uint16_t uart4_rx_dma_buffer_size;
struct ring_buffer uart4_rx_rb;
struct ring_buffer usart3_rx_rb;
uint8_t uart4_rx_rb_data[512];
struct ublox_gnss_cfg_val cfg[10];


/**
 * @brief Updates sensor readings from all onboard sensors
 * @param sensors Pointer to Sensors structure to store updated readings
 * @param huart UART handle for debug output
 */
void update_sensors(Sensors *sensors, UART_HandleTypeDef *huart) {
    float32_t accel_readings[3];
    float32_t gyro_readings[3];
    //double mag_readings[3];
    adis_read_accel(&imu_device, accel_readings);
    sensors->accel_x = -1.0 * accel_readings[0];
    sensors->accel_y = -1.0 * accel_readings[1];
    sensors->accel_z = accel_readings[2];
    adis_read_gyro(&imu_device, gyro_readings);
    sensors->gyro_x = -1.0 * gyro_readings[0] * PI / 180;
    sensors->gyro_y = -1.0 * gyro_readings[1] * PI / 180;
    sensors->gyro_z = gyro_readings[2] * PI / 180;
    MS5607Update();
    uint32_t bytes_to_read = ring_buffer_get_full(&uart4_rx_rb);
    if (bytes_to_read) {
        uint8_t tmp[bytes_to_read];
        uint8_t *tmp2 = tmp;
        size_t bytes_read = ring_buffer_read(&uart4_rx_rb, tmp, bytes_to_read);
        uint8_t msg[256];
        uint16_t msg_len;  // Changed from len to msg_len
        uint8_t *rem;
        uint8_t cls;
        uint8_t id;
        ublox_protocol_decode(tmp2, bytes_read, &cls, &id, msg, sizeof(msg), &msg_len, &rem);
        if (cls == 0x01) { 
            switch (id) {
                case 0x13: { 
                    struct ublox_gnss_nav_hpposecef ecef_data;
                    ublox_gnss_dec_ubx_nav_hpposecef(msg, msg_len, &ecef_data);
                    sensors->gps_offset_x = ecef_data.ecefX * 0.1 + ecef_data.ecefXHp * 0.0001;
                    sensors->gps_offset_y = ecef_data.ecefY * 0.1 + ecef_data.ecefYHp * 0.0001;
                    sensors->gps_offset_z = ecef_data.ecefZ * 0.1 + ecef_data.ecefZHp * 0.0001;
                    break;
                }
                case 0x28: {
                    struct ublox_gnss_nav_hppvt hppvt_data;
                    ublox_gnss_dec_ubx_nav_hppvt(msg, msg_len, &hppvt_data);
                    sensors->gps_x = hppvt_data.lat * 1e-7 + hppvt_data.latHp * 1e-9;
                    sensors->gps_y = hppvt_data.lon * 1e-7 + hppvt_data.lonHp * 1e-9;
                    sensors->gps_z = hppvt_data.height * 1e-3 + hppvt_data.heightHp * 1e-4;
                    break;
                }
            }
        }
    }
}

/**
 * @brief Initializes all onboard sensors and communication interfaces
 * @param sensors Pointer to Sensors structure to initialize
 * @details Initializes IMU, magnetometer, barometer, GPS, and communication interfaces
 */
void sensors_init(Sensors *sensors) {
  imu_device.spi_handle = &hspi4;
  imu_device.cs_pin = GPIOE;
  imu_device.cs_pin_port = GPIO_PIN_4;

  mag_device.spi_handle = &hspi4;  
  mag_device.cs_pin_port = GPIO_PIN_5; 
  mag_device.cs_pin = GPIOC;  
  mag_device.temp_enable = LIS3MDL_TEMP_EN;
  //mag_device.data_rate = LIS3MDL_ODR_40HZ;
  mag_device.self_test = LIS3MDL_SELF_TEST_DIS;
  mag_device.full_scale = LIS3MDL_FS_4Gauss;
  mag_device.z_axis_mode = LIS3MDL_Z_UHP;
  mag_device.endianness = LIS3MDL_LITTLE_ENDIAN;
  lis3mdl_initialize(&mag_device);

  ms5607_state = MS5607_Init(&hspi6, GPIOC, GPIO_PIN_4);
  
  gps.transport_type = UBLOX_GNSS_TRANSPORT_UART;
  //gps.transport_type = UBLOX_GNSS_TRANSPORT_SPI;
  gps.transport_handle.uart = &huart4;
  //gps.transport_handle.spi = &hspi4;
  cfg[0].key_id = 0x10520005; 
  cfg[0].value = 0x01;

  cfg[1].key_id = 0x10740001; 
  cfg[1].value = 0x01; 

  //3D only
  cfg[2].key_id = 0x20110011;
  cfg[2].value = 0x02;

  //UART
  cfg[3].key_id = 0x20910025;
  cfg[3].value = 0x00;

  //Airborn with < 4g acceleration
  cfg[4].key_id = 0x20110021;
  cfg[4].value = 0x08;

  //TimeUTC UART enabled
  cfg[5].key_id = 0x2091005C;
  cfg[5].value = 0x01;

  cfg[6].key_id = 0x20910007;
  cfg[6].value = 0x01;

  cfg[7].key_id = 0x2091002a;
  cfg[7].value = 0x00;

  cfg[8].key_id = 0x40520001;
  cfg[8].value = 38400;  

  ublox_gnss_cfg_val_set_list(&gps, cfg, 10, 0, 1);
  HAL_UARTEx_ReceiveToIdle_IT(&huart4, uart4_rx_dma_buffer, sizeof(uart4_rx_dma_buffer));
  ring_buffer_init(&uart4_rx_rb, uart4_rx_rb_data, sizeof(uart4_rx_rb_data));
  memset(sensors, 0, sizeof(Sensors));
}


/**
 * @brief Initializes all IO peripherals
 * @details Initializes system clock, DMA, I2C, SPI, UART, USB, timers, and GPIO
 */
void protocol_init(void) {
  HAL_Init();
  SystemClock_Config();
  DWT_Init();
  MX_DMA_Init();
  MX_I2C4_Init();
  MX_SPI2_Init();
  MX_SPI4_Init();
  MX_SPI6_Init();
  MX_USART2_UART_Init();
  MX_USB_OTG_HS_PCD_Init();
  MX_USART3_UART_Init();
  MX_UART4_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_GPIO_Init();
}
