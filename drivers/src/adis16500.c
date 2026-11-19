#include "adis16500.h"

#include <string.h>
#include "FreeRTOS.h"
#include "queue.h"
#include <task.h>

#define ADIS_BUFFER_SIZE 34 // 2 command bytes + 16 words (burst read)
#define ADIS_QUEUE_DEPTH 5 // samples that can wait in the queue, 5 is 2.5 ms worth at 2 kHz. bigger survives longer task stalls but costs more

#define ADIS_ACCEL_SCALE (1.0f / 5351254.0f)
#define ADIS_GYRO_SCALE (1.0f / 655360.0f)

#define ADIS_STARTUP_DELAY_US 310000 // imu needs this long after power on
#define ADIS_STALL_DELAY_US 16       // wait between two spi transfers (t_stall)
#define ADIS_CS_SETUP_DELAY_US 1     // wait after CS goes low, t_CS is 200 ns so we round up to 1 us
#define ADIS_SPI_TIMEOUT_MS 10       // longest a blocking spi call waits
#define ADIS_QUEUE_TIMEOUT_MS 10     // longest we wait for a sample, they come every 0.5 ms

#define ADIS_COMBINE_BYTES16(a, b) ((int16_t)((((uint16_t)a) << 8) | ((uint16_t)b))) // 2 bytes (msb first) into a signed 16 bit value
#define ADIS_COMBINE_BYTES32(a, b) ((int32_t)((((uint32_t)(uint16_t)(a)) << 16) | ((uint32_t)(uint16_t)(b)))) // a = OUT word (top 16 bits), b = LOW word (bottom 16 bits, has to stay unsigned)

// DIAG_STAT bits that mean the data can't be trusted: bit 1 (data path overrun), bit 4 (standby, VDD dropped below 2.8 V)
#define ADIS_DIAG_FATAL_MASK ((1 << 1) | (1 << 4))

static adis_device_t* device_ptr;

typedef struct {
    uint32_t timestamp;
    uint8_t rx_buffer[ADIS_BUFFER_SIZE];
} adis_queue_item_t;

static uint8_t tx_buffer[ADIS_BUFFER_SIZE]; // 0x68 then zeros, 0x6800 is the burst command
static adis_queue_item_t rx_item;           // dma writes here, then it gets copied into the queue
static volatile uint32_t pending_timestamp;
static volatile bool dma_start = false;     // DR pulses from power up, so the interrupt does nothing until init is done

static StaticQueue_t queue_struct;
static uint8_t queue_array[ADIS_QUEUE_DEPTH * sizeof(adis_queue_item_t)];
static volatile QueueHandle_t rx_queue = NULL;

// cycle counter, used for the delays and the timestamps
static void DWT_Init(void) {
    if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; 
        DWT->CYCCNT = 0; 
    }
}

static void delay_us(uint32_t microseconds) {
    uint32_t startTick = DWT->CYCCNT;
    uint32_t delayTicks = microseconds * (SystemCoreClock / 1000000);
    
    // handles the counter wrapping around
    while ((uint32_t)(DWT->CYCCNT - startTick) < delayTicks);
}

// a read takes two transfers: send the address, then send zeros and read the data back
static uint16_t adis_read(adis_device_t* device, uint8_t addr) {
    uint8_t tx[2] = {addr, 0x00};
    HAL_GPIO_WritePin(device->cs_pin_port, device->cs_pin, GPIO_PIN_RESET);
    delay_us(ADIS_CS_SETUP_DELAY_US);
    HAL_SPI_Transmit(device->spi_handle, tx, 2, ADIS_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(device->cs_pin_port, device->cs_pin, GPIO_PIN_SET);

    delay_us(ADIS_STALL_DELAY_US);

    uint8_t tx_zero[2] = {0x00, 0x00};
    uint8_t rx[2] = {0x00, 0x00};
    HAL_GPIO_WritePin(device->cs_pin_port, device->cs_pin, GPIO_PIN_RESET);
    delay_us(ADIS_CS_SETUP_DELAY_US);
    HAL_SPI_TransmitReceive(device->spi_handle, tx_zero, rx, 2, ADIS_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(device->cs_pin_port, device->cs_pin, GPIO_PIN_SET);

    delay_us(ADIS_STALL_DELAY_US);

    return (uint16_t)(((uint16_t)rx[0] << 8) | (uint16_t)rx[1]);
}

// a write is two transfers: low byte at addr, then high byte at addr + 1
// the old register value gets OR'd in, so this can set bits but not clear them
static void adis_write(adis_device_t* device, uint8_t addr, uint16_t value) {
    value |= adis_read(device, addr);

    uint8_t low_byte = (uint8_t)(value & 0xFF);
    uint8_t high_byte = (uint8_t)((value >> 8) & 0xFF);
    uint8_t tx_low[2] = {(uint8_t)(addr | (1 << 7)), low_byte};
    uint8_t tx_high[2] = {(uint8_t)((addr + 1) | (1 << 7)), high_byte};

    HAL_GPIO_WritePin(device->cs_pin_port, device->cs_pin, GPIO_PIN_RESET);
    delay_us(ADIS_CS_SETUP_DELAY_US);
    HAL_SPI_Transmit(device->spi_handle, tx_low, 2, ADIS_SPI_TIMEOUT_MS); 
    HAL_GPIO_WritePin(device->cs_pin_port, device->cs_pin, GPIO_PIN_SET);

    delay_us(ADIS_STALL_DELAY_US);

    HAL_GPIO_WritePin(device->cs_pin_port, device->cs_pin, GPIO_PIN_RESET);
    delay_us(ADIS_CS_SETUP_DELAY_US);
    HAL_SPI_Transmit(device->spi_handle, tx_high, 2, ADIS_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(device->cs_pin_port, device->cs_pin, GPIO_PIN_SET);

    delay_us(ADIS_STALL_DELAY_US);
}

bool adis_init(adis_device_t* device, SPI_HandleTypeDef* spi_handle, GPIO_TypeDef* cs_pin_port, uint16_t cs_pin) {
    DWT_Init();

    if (device == NULL) {
        return false;
    }

    memset(&device->status, 0, sizeof(device->status));

    if (spi_handle == NULL || cs_pin_port == NULL) {
        device->status.init_failed = true;
        return false;
    }

    delay_us(ADIS_STARTUP_DELAY_US);
    
    device->spi_handle = spi_handle;
    device->cs_pin_port = cs_pin_port;
    device->cs_pin = cs_pin;

    device_ptr = device;
    memset(tx_buffer, 0x00, ADIS_BUFFER_SIZE);
    tx_buffer[0] = 0x68;
    memset(rx_item.rx_buffer, 0xFF, ADIS_BUFFER_SIZE);

    uint16_t prod_id = adis_read(device, ADIS_PROD_ID);
    if (prod_id != 16500) {
        device->status.init_failed = true;
        return false;
    }

    // bit 9 = 32-bit burst, bit 0 = DR active high
    uint16_t msc_ctrl = (1 << 9) | 1;
    adis_write(device, ADIS_MSC_CTRL, msc_ctrl);

    // read it back to make sure the write worked
    uint16_t msc_check = adis_read(device, ADIS_MSC_CTRL);
    if ((msc_check & msc_ctrl) != msc_ctrl) {
        device->status.init_failed = true;
        return false;
    }

    rx_queue = xQueueCreateStatic(ADIS_QUEUE_DEPTH, sizeof(adis_queue_item_t), queue_array, &queue_struct);

    // lock the spi bus once and never unlock, the generated isr.c only calls our spi callbacks while we hold it
    // (unlock has to happen in the same task, so it can't go in the complete callback)
    if (bus_spi_lock(BUS_SPI_DEVICE_adis16500) == NULL) {
        device->status.init_failed = true;
        return false;
    }

    // from here on the DR interrupt is allowed to start dma
    dma_start = true;
    return true;
}

// DR pulse (rising edge): start the dma read
void adis16500_data_ready_callback(void) {
    // timestamp first so it's as close to the start of the transfer as possible
    uint32_t timestamp = DWT->CYCCNT;

    if (dma_start == true && device_ptr != NULL) {
        // CS stays low until the transfer complete callback
        HAL_GPIO_WritePin(device_ptr->cs_pin_port, device_ptr->cs_pin, GPIO_PIN_RESET);

        HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(device_ptr->spi_handle, tx_buffer, rx_item.rx_buffer, ADIS_BUFFER_SIZE);
        if (status == HAL_OK) {
            pending_timestamp = timestamp;
        } else { // unsuccessful start
            device_ptr->status.counts[ADIS_ERR_DMA_START]++;
            // HAL_BUSY means the last transfer is still running, so leave CS alone
            if (status != HAL_BUSY) {
                // nothing is running, so nobody else will raise CS
                HAL_GPIO_WritePin(device_ptr->cs_pin_port, device_ptr->cs_pin, GPIO_PIN_SET);
            }
        }
    }
}

// dma finished: CS goes high and the data goes in the queue
void adis16500_spi_tx_rx_cplt_callback(void) {
    if (device_ptr == NULL) {
        return;
    }

    HAL_GPIO_WritePin(device_ptr->cs_pin_port, device_ptr->cs_pin, GPIO_PIN_SET);

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (rx_queue != NULL) {
        rx_item.timestamp = pending_timestamp;
        if (xQueueSendToBackFromISR(rx_queue, &rx_item, &xHigherPriorityTaskWoken) != pdTRUE) {
            device_ptr->status.counts[ADIS_ERR_QUEUE_FULL]++;
        }
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// the complete callback doesn't run after an error, so raise CS here too
void adis16500_spi_error_callback(void) {
    if (device_ptr == NULL) {
        return;
    }

    HAL_GPIO_WritePin(device_ptr->cs_pin_port, device_ptr->cs_pin, GPIO_PIN_SET);
    device_ptr->status.counts[ADIS_ERR_SPI]++;
}

bool adis_has_fatal_error(adis_device_t* device) {
    if (device == NULL) {
        return true;
    }
    return device->status.init_failed || device->status.imu_fault;
}

uint32_t adis_get_error_count(adis_device_t* device, adis_err_t type) {
    if (device == NULL || type >= ADIS_ERR_COUNT) return 0;
    return device->status.counts[type];
}

// waits for a sample from the queue and decodes it, returns false if there isn't a good one
bool adis_get_sample(adis_device_t* device, adis_sample_t *sample) {
    if (device == NULL || sample == NULL) {
        return false;
    }

    adis_queue_item_t item;
    const uint8_t* rx_data = item.rx_buffer;
    float32_t raw_gyro[3];
    
    if (rx_queue != NULL) {
        if (xQueueReceive(rx_queue, &item, pdMS_TO_TICKS(ADIS_QUEUE_TIMEOUT_MS)) == true) {
            // bytes 0 and 1 are the command echo, 2 to 31 are data, 32 and 33 are the checksum
            uint16_t checksum = 0;
            for (int i = 2; i < 32; i++) {
                checksum += rx_data[i]; 
            }

            if (checksum != ADIS_COMBINE_BYTES16(rx_data[32], rx_data[33])) {
                 // bad checksum, the sample is dropped
                device->status.counts[ADIS_ERR_CHECKSUM]++;
                return false;
            } else {
                sample->status = (uint16_t)ADIS_COMBINE_BYTES16(rx_data[2], rx_data[3]);

                // for each 32 bit value the LOW word comes first, then the OUT word
                raw_gyro[0] = (float32_t)ADIS_COMBINE_BYTES32(ADIS_COMBINE_BYTES16(rx_data[6], rx_data[7]), ADIS_COMBINE_BYTES16(rx_data[4], rx_data[5])) * ADIS_GYRO_SCALE;
                raw_gyro[1] = (float32_t)ADIS_COMBINE_BYTES32(ADIS_COMBINE_BYTES16(rx_data[10], rx_data[11]), ADIS_COMBINE_BYTES16(rx_data[8], rx_data[9])) * ADIS_GYRO_SCALE;
                raw_gyro[2] = (float32_t)ADIS_COMBINE_BYTES32(ADIS_COMBINE_BYTES16(rx_data[14], rx_data[15]), ADIS_COMBINE_BYTES16(rx_data[12], rx_data[13])) * ADIS_GYRO_SCALE;

                sample->gyro[0] = raw_gyro[0] - device->bias.x_bias;
                sample->gyro[1] = raw_gyro[1] - device->bias.y_bias;
                sample->gyro[2] = raw_gyro[2] - device->bias.z_bias;

                sample->accel[0] = (float32_t)ADIS_COMBINE_BYTES32(ADIS_COMBINE_BYTES16(rx_data[18], rx_data[19]), ADIS_COMBINE_BYTES16(rx_data[16], rx_data[17])) * ADIS_ACCEL_SCALE;
                sample->accel[1] = (float32_t)ADIS_COMBINE_BYTES32(ADIS_COMBINE_BYTES16(rx_data[22], rx_data[23]), ADIS_COMBINE_BYTES16(rx_data[20], rx_data[21])) * ADIS_ACCEL_SCALE;
                sample->accel[2] = (float32_t)ADIS_COMBINE_BYTES32(ADIS_COMBINE_BYTES16(rx_data[26], rx_data[27]), ADIS_COMBINE_BYTES16(rx_data[24], rx_data[25])) * ADIS_ACCEL_SCALE;

                sample->temp = ADIS_COMBINE_BYTES16(rx_data[28], rx_data[29]) * 0.1f;
                sample->data_counter = (uint16_t)ADIS_COMBINE_BYTES16(rx_data[30], rx_data[31]);
                sample->timestamp = item.timestamp;

                // a sample dropped for a bad checksum also counts as a gap
                if (device->status.have_last_counter) {
                    uint16_t missed = (uint16_t)(sample->data_counter - device->status.last_data_counter - 1);
                    device->status.counts[ADIS_ERR_DATA_GAP] += missed;
                }
                device->status.last_data_counter = sample->data_counter;
                device->status.have_last_counter = true;

                if (sample->status & ADIS_DIAG_FATAL_MASK) {
                    device->status.imu_fault = true;
                }
            }

            // the window holds raw gyro values, bias is their average
            if (device->bias.mode) {
                device->bias.x_buffer[device->bias.write_index] = raw_gyro[0];
                device->bias.y_buffer[device->bias.write_index] = raw_gyro[1];
                device->bias.z_buffer[device->bias.write_index] = raw_gyro[2];
                device->bias.write_index = (uint8_t)(device->bias.write_index + 1) % device->bias.buffer_size;

                if (device->bias.sample_counter < device->bias.buffer_size) {
                    device->bias.sample_counter++;
                }

                // no bias until the window has N samples
                if (device->bias.sample_counter >= device->bias.buffer_size) {
                    float32_t x_sum = 0.0f;
                    float32_t y_sum = 0.0f;
                    float32_t z_sum = 0.0f;
                    for (uint8_t i = 0; i < device->bias.buffer_size; i++) {
                        x_sum += device->bias.x_buffer[i];
                        y_sum += device->bias.y_buffer[i];
                        z_sum += device->bias.z_buffer[i];
                    }
                    device->bias.x_bias = x_sum / device->bias.buffer_size;
                    device->bias.y_bias = y_sum / device->bias.buffer_size;
                    device->bias.z_bias = z_sum / device->bias.buffer_size;
                }
            }
        } else {
            // nothing in the queue for 10 ms
            device->status.counts[ADIS_ERR_QUEUE_TIMEOUT]++;
            return false;
        }
    } else {
        return false;
    }

    return true;
}

// turns the sliding window bias on or off and sets the window size N
// changing either one resets the bias to 0
void adis_bias_mode(adis_device_t* device, uint8_t buffer_size_N, bool enable) {
    if (device == NULL || (enable && buffer_size_N == 0)) {
        return;
    }
    
    if (device->bias.mode != enable || device->bias.buffer_size != buffer_size_N) {
        device->bias.mode = enable;
        device->bias.buffer_size = buffer_size_N;
        device->bias.x_bias = 0;
        device->bias.y_bias = 0;
        device->bias.z_bias = 0;
        device->bias.sample_counter = 0;
        device->bias.write_index = 0;
        memset(device->bias.x_buffer, 0, sizeof(device->bias.x_buffer)); // setting all to 0, but we're only really using the first N
        memset(device->bias.y_buffer, 0, sizeof(device->bias.x_buffer));
        memset(device->bias.z_buffer, 0, sizeof(device->bias.x_buffer));
    }
}