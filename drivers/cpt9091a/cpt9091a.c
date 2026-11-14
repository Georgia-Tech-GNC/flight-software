#include "cpt9091a.h"

#include "port.h"

// Timer config, copy from CubeMX directly (don't add 1 to prescaler or period)
#define DMA_TIM_BASE_FREQUENCY_HZ buzzer_tim_clock_base_frequency
#define DMA_TIM_PRESCALER 3
#define DMA_TIM_COUNTER_PERIOD 499

#define DMA_TIM_FREQUENCY_HZ (DMA_TIM_BASE_FREQUENCY_HZ / (DMA_TIM_PRESCALER + 1))

#define DAC_FREQUENCY_HZ (DMA_TIM_FREQUENCY_HZ / (DMA_TIM_COUNTER_PERIOD + 1))
#define DAC_DMA_BUFFER_LENGTH_S 0.01
#define DAC_DMA_BUFFER_LENGTH ((size_t)(DAC_FREQUENCY_HZ * DAC_DMA_BUFFER_LENGTH_S))

static __attribute__((section(".dma"), aligned(32))) uint8_t dma_buffer[DAC_DMA_BUFFER_LENGTH];

typedef struct {
    DAC_HandleTypeDef* dac;
    TIM_HandleTypeDef* tim;
    cpt9091a_fill_next_chunk_callback fill_next_chunk;
} cpt9091a_handle;

static cpt9091a_handle cpt9091a;

static void cpt9091a_half_cmplt_callback(DAC_HandleTypeDef* dac) {
    if (dac == cpt9091a.dac && cpt9091a.fill_next_chunk != NULL) {
        cpt9091a.fill_next_chunk(
            dma_buffer, 
            DAC_DMA_BUFFER_LENGTH / 2, 
            DAC_DMA_BUFFER_LENGTH_S / 2.0
        );
    }
}

static void cpt9091a_cmplt_callback(DAC_HandleTypeDef* dac) {
    if (dac == cpt9091a.dac && cpt9091a.fill_next_chunk != NULL) {
        cpt9091a.fill_next_chunk(
            &dma_buffer[DAC_DMA_BUFFER_LENGTH / 2], 
            DAC_DMA_BUFFER_LENGTH / 2, 
            DAC_DMA_BUFFER_LENGTH_S / 2.0
        );
    }
}

bool cpt9091a_init(DAC_HandleTypeDef* dac, TIM_HandleTypeDef* tim) {
    HAL_TIM_Base_Stop(tim);
    
    cpt9091a.dac = dac;
    cpt9091a.tim = tim;
    
    HAL_StatusTypeDef status;
    status = HAL_DAC_RegisterCallback(cpt9091a.dac, HAL_DAC_CH1_HALF_COMPLETE_CB_ID, cpt9091a_half_cmplt_callback);
    if (status != HAL_OK)
        return false;
    status = HAL_DAC_RegisterCallback(cpt9091a.dac, HAL_DAC_CH1_COMPLETE_CB_ID, cpt9091a_cmplt_callback);
    if (status != HAL_OK)
        return false;
    
    // Fill dma_buffer with 0s
    cpt9091a_play_sin(0.0, 400.0);
    
    status = HAL_DAC_Start_DMA(
        cpt9091a.dac, 
        DAC1_CHANNEL_1, 
        (uint32_t*)dma_buffer, 
        DAC_DMA_BUFFER_LENGTH, 
        DAC_ALIGN_8B_R
    );
    
    if (status != HAL_OK)
        return false;
    
    HAL_TIM_Base_Start(cpt9091a.tim);
    
    return true;
}

void cpt9091a_deinit() {
    HAL_TIM_Base_Stop(cpt9091a.tim);
    HAL_DAC_Stop_DMA(cpt9091a.dac, DAC1_CHANNEL_1);
    
    cpt9091a.fill_next_chunk = NULL;
    cpt9091a.dac = NULL;
    cpt9091a.tim = NULL;
}

void cpt9091a_play_square(double volume, double frequency) {
    HAL_TIM_Base_Stop(cpt9091a.tim);
    
    cpt9091a.fill_next_chunk = NULL;
    for (uint32_t i = 0; i < DAC_DMA_BUFFER_LENGTH; i++) 
    {
        const double time = (1.0 / DAC_FREQUENCY_HZ) * i;
        dma_buffer[i] = (uint8_t)(255.0 * volume * (copysign(0.5, sin(2.0 * M_PI * frequency * time)) + 0.5));
    }
    
    HAL_TIM_Base_Start(cpt9091a.tim);
}

void cpt9091a_play_sin(double volume, double frequency) {
    HAL_TIM_Base_Stop(cpt9091a.tim);
    
    cpt9091a.fill_next_chunk = NULL;
    for (uint32_t i = 0; i < DAC_DMA_BUFFER_LENGTH; i++) 
    {
        const double time = (1.0 / DAC_FREQUENCY_HZ) * i;
        dma_buffer[i] = (uint8_t)(255.0 * volume * (sin(2.0 * M_PI * frequency * time) + 1) / 2.0);
    }
    
    HAL_TIM_Base_Start(cpt9091a.tim);
}

void cpt9091a_play(cpt9091a_fill_next_chunk_callback fill_next_chunk) {
    HAL_TIM_Base_Stop(cpt9091a.tim);
    
    cpt9091a.fill_next_chunk = fill_next_chunk;
    
    HAL_TIM_Base_Start(cpt9091a.tim);
}

// void cpt9091a_play_notes(DAC_HandleTypeDef* cpt9091a, double volume, CPT9091A_Note* notes, size_t size)
// {
//     for (size_t i = 0; i < size; i++) 
//     {
//         cpt9091a_play_sin(cpt9091a, volume, notes[i].frequency);
//         uint32_t start_time = HAL_GetTick();
//     }
// }
// 
// void cpt9091a_play_pattern_1(DAC_HandleTypeDef* cpt9091a, double volume)
// {
//     CPT9091A_Note notes[12];
//     notes[0].frequency = 659.25;
//     notes[0].duration = 0.15;
// 
//     notes[1].frequency = 659.25;
//     notes[1].duration = 0.15;
// 
//     notes[2].frequency = 0.0;
//     notes[2].duration = 0.15;
// 
//     notes[3].frequency = 659.25;
//     notes[3].duration = 0.15;
// 
//     notes[4].frequency = 0.0;
//     notes[4].duration = 0.15;
// 
//     notes[5].frequency = 523.25;
//     notes[5].duration = 0.15;
// 
//     notes[6].frequency = 659.25;
//     notes[6].duration = 0.30;
// 
//     notes[7].frequency = 0.0;
//     notes[7].duration = 0.15;
// 
//     notes[8].frequency = 783.99;
//     notes[8].duration = 0.30;
// 
//     notes[9].frequency = 0.0;
//     notes[9].duration = 0.45;
// 
//     notes[10].frequency = 392.00;
//     notes[10].duration = 0.30;
// 
//     notes[11].frequency = 0.0;
//     notes[11].duration = 0.45;
//     
//     cpt9091a_play_notes(cpt9091a, volume, notes, 12);
// }
