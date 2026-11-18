#include "gimbal_tests.h"

#include "FreeRTOS.h"
#include "task.h"
#include "servo.h"
#include "encoder.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "port.h"
#include "nanoprintf.h"

extern UART_HandleTypeDef debug_uart;
#define TWO_PI 6.2831853071795864769f

// #define SAMPLE_RATE_HZ 3000U
// #define MAX_SAMPLES 60000U

// static gimbal_sample_t samples[MAX_SAMPLES];
// static uint32_t sample_count = 0;

// typedef struct {
//     uint32_t timestamp_ticks;
//     float command_deg;
//     float encoder_deg;
// } gimbal_sample_t;


// static bool record_sample(gimbal_test_context_t *ctx, TickType_t test_start, float command_deg) {
//     if (sample_count >= MAX_SAMPLES) {
//         return false;
//     }

//     float encoder_deg;

//     uint32_t timestamp = (uint32_t)(xTaskGetTickCount() - test_start);

//     if (!encoder_get_angle(ctx->encoder, &encoder_deg)) {
//         return false;
//     }

//     samples[sample_count].timestamp_ticks = timestamp;
//     samples[sample_count].command_deg = command_deg;
//     samples[sample_count].encoder_deg = encoder_deg;

//     sample_count++;

//     return true;
// }
// static void transmit_samples(void)
// {
//     char message[128];

//     const char header[] =
//         "timestamp_s,command_deg,encoder_deg\r\n";

//     HAL_UART_Transmit(
//         &debug_uart,
//         (uint8_t *)header,
//         sizeof(header) - 1,
//         HAL_MAX_DELAY
//     );

//     for (uint32_t i = 0; i < sample_count; i++) {

//         float timestamp_s =
//             (float)samples[i].timestamp_ticks /
//             (float)configTICK_RATE_HZ;

//         int len = npf_snprintf(
//             message,
//             sizeof(message),
//             "%.4f,%.2f,%.2f\r\n",
//             (double)timestamp_s,
//             (double)samples[i].command_deg,
//             (double)samples[i].encoder_deg
//         );

//         if (len > 0 && len < (int)sizeof(message)) {
//             HAL_UART_Transmit(
//                 &debug_uart,
//                 (uint8_t *)message,
//                 (uint16_t)len,
//                 HAL_MAX_DELAY
//             );
//         }
//     }
// }

#define CHUNK_SIZE 10000

typedef struct {
    uint32_t timestamp_ticks;
    float command_deg;
    float encoder_deg;
} gimbal_sample_t;

static gimbal_sample_t chunk[CHUNK_SIZE];
static uint32_t chunk_count = 0;

static void transmit_chunk(void)
{
    HAL_UART_Transmit(&debug_uart, (uint8_t *) "Starting data trasmision...\r\n", 30, HAL_MAX_DELAY);

    char message[128];

    for (uint32_t i = 0; i < chunk_count; i++) {

        float timestamp_s =
            (float)chunk[i].timestamp_ticks /
            (float)configTICK_RATE_HZ;

        int len = npf_snprintf(
            message,
            sizeof(message),
            "%.4f,%.2f,%.2f\r\n",
            (double)timestamp_s,
            (double)chunk[i].command_deg,
            (double)chunk[i].encoder_deg
        );

        if (len > 0 && len < (int)sizeof(message)) {
            HAL_UART_Transmit(
                &debug_uart,
                (uint8_t *)message,
                (uint16_t)len,
                HAL_MAX_DELAY
            );
        }
    }

    chunk_count = 0;
}

static bool record_sample(gimbal_test_context_t *ctx,TickType_t test_start, float command_deg) {
    float encoder_deg;

    if (!encoder_get_angle(ctx->encoder, &encoder_deg)) {
        return false;
    }
    
    chunk[chunk_count].timestamp_ticks = (uint32_t)(xTaskGetTickCount() - test_start);

    chunk[chunk_count].command_deg = command_deg;
    chunk[chunk_count].encoder_deg = encoder_deg;

    chunk_count++;

    if (chunk_count == CHUNK_SIZE) {
        transmit_chunk();
    }

    return true;
}

/**
 * Doublet command sequence.
 *
 * Commands 0, +amplitude, 0, and -amplitude degrees,
 * holding each position for the configured duration.
 * Repeats the sequence for the configured number of cycles.
 *
 * @param ctx Pointer to the gimbal test context.
 * @param amplitude_deg Maximum displacement from zero, in degrees.
 */
static void run_doublet(gimbal_test_context_t *ctx,float amplitude_deg) {
    const float positions[] = {0.0f,amplitude_deg,0.0f,-amplitude_deg};

    char message[128];

    npf_snprintf(
        message,
        sizeof(message),
        "\r\nStarting doublet test: +/-%.2f deg | %lu cycles\r\n",
        (double)amplitude_deg,
        (unsigned long)DOUBLET_CYCLES
    );

    HAL_UART_Transmit(&debug_uart,(uint8_t *)message,(uint16_t)strlen(message),HAL_MAX_DELAY);

    TickType_t test_start = xTaskGetTickCount();

    for (uint32_t cycle = 0; cycle < DOUBLET_CYCLES; cycle++) {
        for (uint32_t step = 0; step < 4; step++) {

            float command_deg = positions[step];

            if (!servo_set_pos(ctx->servo, command_deg)) {
                return;
            }

            TickType_t start = xTaskGetTickCount();
            TickType_t last_wake = start;

            
            while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(DOUBLET_HOLD_MS)) {

                // float degrees;

                // uint32_t timestamp_ms = (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount() - test_start);

                // bool valid = encoder_get_angle(ctx->encoder,&degrees);

                // if (valid) {
                //     npf_snprintf(message,sizeof(message),
                //         "Time: %lu ms | Command: %.2f deg | Measured Angle: %.2f deg\r\n",
                //         (unsigned long)timestamp_ms,
                //         (double)command_deg,
                //         (double)degrees
                //     );
                // 
                //     HAL_UART_Transmit(&debug_uart,(uint8_t *)message,(uint16_t)strlen(message),HAL_MAX_DELAY);
                // }

                record_sample(ctx, test_start, command_deg);
                vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TEST_SAMPLE_MS));
            }
        }   
        // transmit_chunk();
    }
}

/**
 * Run Test #1: Doublet response without payload.
 *
 * Executes five small doublet cycles (+/-2 degrees),
 * followed by five large doublet cycles (+/-theta_max).
 *
 * @param params Pointer to the gimbal test context.
 */
void gimbal_test_1_task(void *params) {
    gimbal_test_context_t *ctx = (gimbal_test_context_t *)params;
    // Return to zero position
    servo_set_pos(ctx->servo, 0.0f);

    // Small doublet response
    encoder_set_zero(ctx->encoder);
    run_doublet(ctx, SMALL_STEP_DEG);
    transmit_chunk();

    // Return to zero position
    servo_set_pos(ctx->servo, 0.0f);

    // Large doublet response
    encoder_set_zero(ctx->encoder);
    run_doublet(ctx, THETA_MAX_DEG);
    transmit_chunk();

    // Return to zero position
    servo_set_pos(ctx->servo, 0.0f);

    // Delete the task after completion
    vTaskDelete(NULL);
}


/**
 * Hold a servo position while measuring the encoder.
 *
 * Commands the requested angle and transmits the encoder
 * pulse width over UART every TEST_SAMPLE_MS milliseconds.
 *
 * @param ctx Pointer to the gimbal test context.
 * @param angle_deg Commanded servo angle in degrees.
 * @param test_start Start tick of the staircase test.
 *
 * @return true if the servo command succeeds, false otherwise.
 */
static bool staircase_step(gimbal_test_context_t *ctx,float angle_deg,TickType_t test_start) {
    if (!servo_set_pos(ctx->servo, angle_deg)) {
        return false;
    }

    TickType_t step_start = xTaskGetTickCount();
    TickType_t last_wake = step_start;

    const TickType_t sample_ticks = pdMS_TO_TICKS(TEST_SAMPLE_MS);
    const TickType_t hold_ticks = pdMS_TO_TICKS(STAIRCASE_HOLD_MS);

    if (sample_ticks == 0 || hold_ticks == 0) {
        return false;
    }

    while ((xTaskGetTickCount() - step_start) < hold_ticks) {

        // float degrees;
        // char message[128];

        // uint32_t timestamp_ms = (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount() - test_start);

        // bool valid = encoder_get_angle(ctx->encoder,&degrees);

        // if (valid) {
        //     npf_snprintf(
        //         message,
        //         sizeof(message),
        //         "Time: %lu ms | Command: %.2f deg | Measured Angle: %.2f deg\r\n",
        //         (unsigned long)timestamp_ms,
        //         (double)angle_deg,
        //         (double)degrees
        //     );

        //     HAL_UART_Transmit(&debug_uart,(uint8_t *)message,(uint16_t)strlen(message),HAL_MAX_DELAY);
        // } 
        record_sample(ctx, test_start, angle_deg);

        vTaskDelayUntil(&last_wake, sample_ticks);
    }
    // transmit_chunk();

    return true;
}

/**
 * Run the staircase response test.
 *
 * Moves from 0 to +5 degrees, then to -5 degrees,
 * and finally returns to 0 degrees in 0.5-degree steps.
 * Holds each position for 500 milliseconds.
 *
 * @param ctx Pointer to the gimbal test context.
 *
 * @return true if all commands succeed, false otherwise.
 */
static bool run_staircase(gimbal_test_context_t *ctx) {
    char message[] = "\r\nStarting staircase test: +/-5 deg | Step: 0.5 deg\r\n";

    HAL_UART_Transmit(&debug_uart,(uint8_t *)message,(uint16_t)strlen(message),HAL_MAX_DELAY);

    TickType_t test_start = xTaskGetTickCount();

    // 0 to +5 degrees
    for (int32_t i = 0; i <= 10; i++) {
        if (!staircase_step(ctx, (float)i * STAIRCASE_STEP_DEG, test_start)) {
            return false;
        }
    }
    transmit_chunk();

    // +4.5 to -5 degrees
    for (int32_t i = 9; i >= -10; i--) {
        if (!staircase_step(ctx, (float)i * STAIRCASE_STEP_DEG,test_start)) {
            return false;
        }
    }
    transmit_chunk();

    // -4.5 to 0 degrees
    for (int32_t i = -9; i <= 0; i++) {
        if (!staircase_step(ctx, (float)i * STAIRCASE_STEP_DEG, test_start)) {
            return false;
        }
    }
    transmit_chunk();

    return true;
}

/**
 * Run Test #3: Staircase response for backlash.
 *
 * Executes a staircase sequence from 0 to +5 degrees,
 * then to -5 degrees, and finally returns to zero.
 *
 * @param params Pointer to the gimbal test context.
 */
void gimbal_test_3_task(void *params) {
    gimbal_test_context_t *ctx = (gimbal_test_context_t *)params;
    servo_set_pos(ctx->servo, 0.0f);
    encoder_set_zero(ctx->encoder);

    run_staircase(ctx);

    // Return to zero position
    servo_set_pos(ctx->servo, 0.0f);

    vTaskDelete(NULL);
}

/**
 * Run a sinusoidal chirp response test.
 *
 * Commands a sine wave with constant amplitude and
 * linearly increasing frequency from SWEEP_START_HZ
 * to SWEEP_END_HZ over SWEEP_DURATION_S seconds.
 *
 * Samples encoder pulse width and transmits measurements
 * over UART every TEST_SAMPLE_MS milliseconds.
 *
 * @param ctx Pointer to the gimbal test context.
 *
 * @return true if all servo commands succeed, false otherwise.
 */
static bool run_chirp(gimbal_test_context_t *ctx) {
    char message[128];

    npf_snprintf(
        message,
        sizeof(message),
        "\r\nStarting chirp: %.2f-%.2f Hz | Amplitude: +/-%.2f deg | Duration: %.2f s\r\n",
        (double)SWEEP_START_HZ,
        (double)SWEEP_END_HZ,
        (double)SWEEP_AMPLITUDE_DEG,
        (double)SWEEP_DURATION_S
    );

    HAL_UART_Transmit(&debug_uart,(uint8_t *)message,(uint16_t)strlen(message),HAL_MAX_DELAY);

    const TickType_t sample_ticks = pdMS_TO_TICKS(TEST_SAMPLE_MS);

    if (sample_ticks == 0) {
        return false;
    }

    const float sweep_rate = (SWEEP_END_HZ - SWEEP_START_HZ) / SWEEP_DURATION_S;

    TickType_t test_start = xTaskGetTickCount();
    TickType_t last_wake = test_start;

    while (true) {
        TickType_t now = xTaskGetTickCount();

        uint32_t timestamp_ms = (uint32_t)pdTICKS_TO_MS(now - test_start);

        float t = (float)timestamp_ms / 1000.0f;

        if (t >= SWEEP_DURATION_S) {
            break;
        }

        // Calculate instantaneous chirp phase
        float phase = TWO_PI * (SWEEP_START_HZ * t + 0.5f * sweep_rate * t * t);

        float command_deg = SWEEP_AMPLITUDE_DEG * sinf(phase);

        if (!servo_set_pos(ctx->servo, command_deg)) {
            return false;
        }

        // float degrees;
        // bool valid = encoder_get_angle(ctx->encoder,&degrees);

        // if (valid) {
        //     npf_snprintf(
        //         message,
        //         sizeof(message),
        //         "Time: %lu ms | Command: %.2f deg | Measured Angle: %.2f deg\r\n",
        //         (unsigned long)timestamp_ms,
        //         (double)command_deg,
        //         (double)degrees
        //     );

        //     HAL_UART_Transmit(&debug_uart,(uint8_t *)message,(uint16_t)strlen(message),HAL_MAX_DELAY);
        // } 
        record_sample(ctx, test_start, command_deg);

        vTaskDelayUntil(&last_wake, sample_ticks);
    }

    transmit_chunk();
    return true;
}

/**
 * Run Test #4: Sinusoidal chirp response.
 *
 * Sweeps from 0.5 Hz to 15 Hz over 20 seconds,
 * commanding a constant +/-2-degree amplitude.
 *
 * @param params Pointer to the gimbal test context.
 */
void gimbal_test_4_task(void *params) {
    gimbal_test_context_t *ctx = (gimbal_test_context_t *)params;
    servo_set_pos(ctx->servo, 0.0f);
    encoder_set_zero(ctx->encoder);
    
    run_chirp(ctx);

    // Return to zero position
    servo_set_pos(ctx->servo, 0.0f);

    vTaskDelete(NULL);
}