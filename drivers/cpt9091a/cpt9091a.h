#pragma once

#include "main.h"

#define CPT9091A_FREQUENCY_LOW 400.0
#define CPT9091A_FREQUENCY_MED 800.0
#define CPT9091A_FREQUENCY_HIGH 1200.0

typedef struct {
    DAC_HandleTypeDef* dac;
    TIM_HandleTypeDef* tim;
} CPT9091A;

bool cpt9091a_init(CPT9091A* cpt9091a);
void cpt9091a_play_square(CPT9091A* cpt9091a, double volume, double frequency);
void cpt9091a_play_sin(CPT9091A* cpt9091a, double volume, double frequency);
// void cpt9091a_play_notes(DAC_HandleTypeDef* cpt9091a, double volume, CPT9091A_Note* notes, size_t size);
/** Plays "The Iconic Intro Sequence" */
// void cpt9091a_play_pattern_1(DAC_HandleTypeDef* cpt9091a, double volume);
