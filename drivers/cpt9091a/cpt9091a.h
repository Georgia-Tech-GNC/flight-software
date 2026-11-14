#pragma once

#include "main.h"

/**
 * A function pointer to a function that fills the next chunk of audio samples in the data buffer.
 * @param data a pointer to an array of uint8_t representing audio samples.
 * @param size the length of the array to fill.
 * @param length_s the length of the array to fill in seconds.
 */
typedef void (*cpt9091a_fill_next_chunk_callback)(uint8_t* data, uint32_t size, double length_s);

/**
 * 
 * @param dac the handle to the DAC associated with the buzzer.
 * @param tim the timer handle that controls the DMA transfers.
 * @return true if the device was successfully initialized.
 */
bool cpt9091a_init(DAC_HandleTypeDef* dac, TIM_HandleTypeDef* tim);

/**
 * Stops the DMA transfer and stops the timer.
 */
void cpt9091a_deinit();

/**
 * Play a square wave to the buzzer.
 * @param volume the volume of the square wave (0 = 0V -> 1 = 3.3V).
 * @param frequency the frequency of the wave.
 */
void cpt9091a_play_square(double volume, double frequency);

/**
 * Play a sin wave to the buzzer.
 * @param volume the volume of the sin wave (0 = 0V -> 1 = 3.3V).
 * @param frequency the frequency of the wave.
 */
void cpt9091a_play_sin(double volume, double frequency);

/**
 * Play an arbitrary signal to the buzzer.
 * @param fill_next_chunk a pointer to cpt9091a_fill_next_chunk_callback
 */
void cpt9091a_play(cpt9091a_fill_next_chunk_callback fill_next_chunk);

// void cpt9091a_play_notes(DAC_HandleTypeDef* cpt9091a, double volume, CPT9091A_Note* notes, size_t size);
/** Plays "The Iconic Intro Sequence" */
// void cpt9091a_play_pattern_1(DAC_HandleTypeDef* cpt9091a, double volume);
