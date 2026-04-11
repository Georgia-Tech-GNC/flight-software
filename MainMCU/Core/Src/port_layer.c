#include "port_layer.h"
#include "packet_encode.h"

// Declare Tasks
task_data_t g_master_task;
task_data_t g_telemetry_task;
task_data_t g_state_flash_task;
task_data_t telem_rx_task;


// Declare stream buffers and mutexes
message_buffer_64_bit_t g_state_message_buffer;
stream_buffer_64_bit_t g_telemetry_command_stream_buffer;
stream_buffer_64_bit_t tx_stream_buffer;


semaphore_data_t g_state_lock;

// Declare global rocket state struct
RocketState g_current_state = {0};

// Local buffers for UART lines
uint8_t state_uart_rx_buf[64];
uint8_t telemetry_uart_rx_buf[64];

uint8_t packet_buffer[260];
uint8_t payload_buffer[260];
int packet_buffer_size = 0;
uint8_t last_cmd_counter[10] = {255, 255, 255, 255, 255, 255, 255, 255, 255, 255};

void rx_telem_handler(void* args) {
    while (1) {
        uint8_t byte; 
        if (xStreamBufferReceive(tx_stream_buffer.handle, &byte, 1, portMAX_DELAY) == 0) continue;

        packet_buffer_size = process_incoming_byte(byte, packet_buffer, packet_buffer_size);

        if (packet_buffer_size < 0) {
            size_t msg_id = extract_packet(packet_buffer, -packet_buffer_size, payload_buffer);
            if (msg_id == COMMAND_MSG_ID) {
                // CMD!
                struct CommandStruct cmd;
                extract_command(payload_buffer, 2, &cmd);
                if (cmd.command_id < 10 && cmd.command_counter != last_cmd_counter[cmd.command_id]) {
                    //char asdsad[40];
                    //int fdsf = sprintf(asdsad, "Stream: %d %d\r\n", cmd.command_id, cmd.command_counter);
                    //HAL_UART_Transmit(&debug_uart, asdsad, fdsf, HAL_MAX_DELAY);
                    xStreamBufferSend(g_telemetry_command_stream_buffer.handle, &cmd.command_id, 1, portMAX_DELAY);
                    last_cmd_counter[cmd.command_id] = cmd.command_counter;
                } else {
                    //char asdsad[40];
                    //int fdsf = sprintf(asdsad, "Fail: %d %d\r\n", cmd.command_id, cmd.command_counter);
                    //HAL_UART_Transmit(&debug_uart, asdsad, fdsf, HAL_MAX_DELAY);
                }

                uint8_t payload[2];
                payload[0] = cmd.command_id; // command id
                payload[1] = cmd.command_counter; // command uuid

                
                uint8_t ack_packet[7];
                size_t sz = generate_packet(payload, 2, ack_packet, COMMAND_ACK_MSG_ID);
                HAL_UART_Transmit(&telemetry_uart, ack_packet, sz, HAL_MAX_DELAY);

                /*
                char buffer[125];
                size_t blem = sprintf(buffer, "len %d %d: %x %x %x %x %x %x %x\r\n", sz, cmd.command_id, ack_packet[0], ack_packet[1], ack_packet[2], ack_packet[3], ack_packet[4], ack_packet[5], ack_packet[6]);
                HAL_UART_Transmit(&debug_uart, buffer, blem, HAL_MAX_DELAY);
                */
            }
            
            packet_buffer_size = 0;
        }
    }
}


/**
 * @brief Initialize the application
 * @return 1 if successful, 0 otherwise
 */
int port_init(void) {
    HAL_UART_Transmit(&debug_uart, "starting\r\n", 10, HAL_MAX_DELAY);

    // Create mutexes
    g_state_lock.handle = xSemaphoreCreateMutexStatic(&g_state_lock.freertos_internal_data);
    if (g_state_lock.handle == NULL) return 0;
    
    // Create all tasks
    g_master_task.handle = xTaskCreateStatic(
        master_task_handler, "master_task", TASK_STACK_SIZE, NULL, 2, g_master_task.stack, &g_master_task.freertos_internal_data
    );
    if (g_master_task.handle == NULL) return 0;

    g_telemetry_task.handle = xTaskCreateStatic(
        telemetry_task_handler, "telemetry_task", TASK_STACK_SIZE, NULL, 1, g_telemetry_task.stack, &g_telemetry_task.freertos_internal_data
    );
    if (g_telemetry_task.handle == NULL) return 0;

    telem_rx_task.handle = xTaskCreateStatic(
        rx_telem_handler, "rx_task", TASK_STACK_SIZE, NULL, 2, telem_rx_task.stack, &telem_rx_task.freertos_internal_data
    );
    if (telem_rx_task.handle == NULL) return 0;

    g_state_flash_task.handle = xTaskCreateStatic(
        state_flash_task, "state_flash_task", TASK_STACK_SIZE, NULL, 1, g_state_flash_task.stack, &g_state_flash_task.freertos_internal_data
    );
    if (g_state_flash_task.handle == NULL) return 0;

    // Create stream buffers
    g_state_message_buffer.handle = xMessageBufferCreateStatic(
        64, g_state_message_buffer.internal_buffer, &g_state_message_buffer.freertos_internal_data);
    if (g_state_message_buffer.handle == NULL) return 0;
    g_telemetry_command_stream_buffer.handle = xStreamBufferCreateStatic(
        64, 1, g_telemetry_command_stream_buffer.internal_buffer, &g_telemetry_command_stream_buffer.freertos_internal_data);
    if (g_telemetry_command_stream_buffer.handle == NULL) return 0;
    tx_stream_buffer.handle = xStreamBufferCreateStatic(
        64, 1, tx_stream_buffer.internal_buffer, &tx_stream_buffer.freertos_internal_data);
    if (tx_stream_buffer.handle == NULL) return 0;

    // Start the UART Interrupt handlers
    if (HAL_UARTEx_ReceiveToIdle_IT(&state_uart, state_uart_rx_buf, 16) != HAL_OK) {
        return 0;
    }
    if (HAL_UARTEx_ReceiveToIdle_IT(&telemetry_uart, telemetry_uart_rx_buf, 7) != HAL_OK) {
        return 0;
    }

    // Initialize and test SD and flash chip
    HAL_UART_Transmit(&debug_uart, "p2\r\n", 4, HAL_MAX_DELAY);



    // Initialize servos
    servo_init(&servo_1, PWM2_TIMER, PWM2_CHANNEL);
    servo_set_pos(&servo_1, 1500);

    servo_init(&servo_2, PWM3_TIMER, PWM3_CHANNEL);
    servo_set_pos(&servo_2, 1500);

    // Initialize rocket state
    g_current_state.orientation = (quaternion_t) {.w = 1.0, .x = 0.0, .y = 0.0, .z = 0.0};
    g_current_state.timestamp = 0;
    g_current_state.servo_cmd_1 = 0;
    g_current_state.servo_cmd_2 = 0;
    g_current_state.w_x = 0;
    g_current_state.w_y = 0;
    g_current_state.w_z = 0;
    g_current_state.state = GROUND;
    
    // Give the StateEstimation core enough time to jumpstart
    HAL_UART_Transmit(&debug_uart, "p3\r\n", 4, 10);
    HAL_UART_Transmit(&debug_uart, "p4\r\n", 4, 10);


    return 1;
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
    /* FreeRTOS boilerplate */
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* Direct uart data to the appropriate place */
    if (huart->Instance == telemetry_uart.Instance) {
        /* Send telemetry data to the telemetry_rx task */
        xStreamBufferSendFromISR(tx_stream_buffer.handle, telemetry_uart_rx_buf, size, &xHigherPriorityTaskWoken);
        
        HAL_UARTEx_ReceiveToIdle_IT(&telemetry_uart, telemetry_uart_rx_buf, 7);
        
    } else if (huart->Instance == state_uart.Instance) {
        xMessageBufferSendFromISR(g_state_message_buffer.handle, state_uart_rx_buf, size, &xHigherPriorityTaskWoken);
        HAL_UARTEx_ReceiveToIdle_IT(&state_uart, state_uart_rx_buf, 16);
    }

    /* FreeRTOS boilerplate */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief Start the application
 */
void port_start(void) {

    // Signal to the state estimation MCU to begin
    //HAL_Delay(2000);
    HAL_UART_Transmit(&state_uart, "r", 1, HAL_MAX_DELAY);
    //HAL_TIM_PWM_Start(PWM0_TIMER, PWM0_CHANNEL);

    //uint16_t count = 1000.0 * 3.2;
    //__HAL_TIM_SET_COMPARE(PWM0_TIMER, PWM0_CHANNEL, count);

    HAL_UART_Transmit(&debug_uart, "starting scheduler\r\n", 20, HAL_MAX_DELAY);

    vTaskStartScheduler();
}