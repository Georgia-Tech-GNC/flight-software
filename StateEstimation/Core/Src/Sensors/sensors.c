#include "sensors.h"
#include "gen_constants.h"

I2C_HandleTypeDef hi2c4;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi4;
SPI_HandleTypeDef hspi6;

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

#define ADIS_SWAP_BYTES(a, b) ((int16_t)((((uint16_t)a) << 8) | (((uint16_t)b) & 0xFF)))

int adis_burst_read(struct ADIS_Device *device, Sensors* sensors) {
    // Lower the CS pin to start the read and wait >200ns
    HAL_GPIO_WritePin((GPIO_TypeDef*) device->cs_pin, (uint16_t)device->cs_pin_port, GPIO_PIN_RESET);
    delay_us(3);

    // Send address 0x6800 to indicate start of burst read
	uint8_t txbuf[22] = {0x68, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t rxbuf[22] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    HAL_SPI_TransmitReceive((SPI_HandleTypeDef*)device->spi_handle, txbuf, rxbuf, 22, 2000);

    HAL_GPIO_WritePin((GPIO_TypeDef*) device->cs_pin, (uint16_t)device->cs_pin_port, GPIO_PIN_SET);
    delay_us(5);

    uint16_t checksum = 0;
    for (int i = 2; i < 20; i++) {
        checksum += rxbuf[i];
    }
    if (checksum != ADIS_SWAP_BYTES(rxbuf[20], rxbuf[21])) {
        char debug[128];
        sensors->imu_status = 0xFFFF;
        int sz = sprintf(debug, "ERR: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\r\n", 
			rxbuf[0], rxbuf[1], rxbuf[2], rxbuf[3], rxbuf[4], rxbuf[5], rxbuf[6], rxbuf[7], rxbuf[8], rxbuf[9],   
            rxbuf[10], rxbuf[11], rxbuf[12], rxbuf[13], rxbuf[14], rxbuf[15], rxbuf[16], rxbuf[17], rxbuf[18], rxbuf[19],   
            rxbuf[20], rxbuf[21]
		);
		HAL_UART_Transmit(&huart3, (uint8_t*)debug, sz, HAL_MAX_DELAY);
        return 1;
    } else {
        //char debug[128];
        //sensors->imu_status = 0xFFFF;
        //int sz = sprintf(debug, "Good: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\r\n", 
		//	rxbuf[0], rxbuf[1], rxbuf[2], rxbuf[3], rxbuf[4], rxbuf[5], rxbuf[6], rxbuf[7], rxbuf[8], rxbuf[9],   
        //    rxbuf[10], rxbuf[11], rxbuf[12], rxbuf[13], rxbuf[14], rxbuf[15], rxbuf[16], rxbuf[17], rxbuf[18], rxbuf[19],   
        //    rxbuf[20], rxbuf[21]
		//);
		//HAL_UART_Transmit(&huart3, (uint8_t*)debug, sz, HAL_MAX_DELAY);

        sensors->imu_status = ADIS_SWAP_BYTES(rxbuf[2], rxbuf[3]);
        sensors->gyro_x = (float)ADIS_SWAP_BYTES(rxbuf[4], rxbuf[5]) *  0.1f;
        sensors->gyro_y = (float)ADIS_SWAP_BYTES(rxbuf[6], rxbuf[7]) * 0.1f;
        sensors->gyro_z = (float)ADIS_SWAP_BYTES(rxbuf[8], rxbuf[9]) * 0.1f;
        sensors->accel_x = (float)ADIS_SWAP_BYTES(rxbuf[10], rxbuf[11]) * 0.01225f;
        sensors->accel_y = (float)ADIS_SWAP_BYTES(rxbuf[12], rxbuf[13]) * 0.01225f;
        sensors->accel_z = (float)ADIS_SWAP_BYTES(rxbuf[14], rxbuf[15]) * 0.01225f;
        return 0;
    }
	
}

int update_sensors(Sensors *sensors) {
    
    //float32_t accel_readings[3];
    //float32_t gyro_readings[3];
    //double mag_readings[3];

    //char buffer[128];
    //MS5607Update();
    //size_t sz = sprintf(buffer, "Baro: %d -> %.4f\r\n", MS5607GetPressurePa(), MS5607GetTemperatureC());
    //HAL_UART_Transmit(&huart3, buffer, sz, HAL_MAX_DELAY);

    return adis_burst_read(&imu_device, sensors);

    /*
    adis_read_accel(&imu_device, accel_readings);
    sensors->accel_x = -1.0 * accel_readings[0];
    sensors->accel_y = -1.0 * accel_readings[1];
    sensors->accel_z = accel_readings[2];
    
    adis_read_gyro(&imu_device, gyro_readings);
    sensors->gyro_x = -1.0 * gyro_readings[0] * PI / 180;
    sensors->gyro_y = -1.0 * gyro_readings[1] * PI / 180;
    sensors->gyro_z = gyro_readings[2] * PI / 180;
    sensors->imu_status = adis_read_status(&imu_device);
    sensors->imu_serial = adis_read_serial_number(&imu_device);
    */
    /*
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
    */
}

void sensors_init(Sensors *sensors) {
  imu_device.spi_handle = &hspi4;
  imu_device.cs_pin = GPIOE;
  imu_device.cs_pin_port = GPIO_PIN_4;

  memset(sensors, 0, sizeof(Sensors));

  // ms5607_state = MS5607_Init(&hspi6, GPIOB, GPIO_PIN_12);

    //char buffer[128];
    //size_t sz = sprintf(buffer, "Baro Status: %d\r\n", ms5607_state);
    //HAL_UART_Transmit(&huart3, buffer, sz, HAL_MAX_DELAY);

  /*
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
  */

  
  /*
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
  */
}