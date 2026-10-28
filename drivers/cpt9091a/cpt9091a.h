#pragma once

#include "main.h"

#define CPT9091A_FREQUENCY_LOW 400.0
#define CPT9091A_FREQUENCY_MED 800.0
#define CPT9091A_FREQUENCY_HIGH 1200.0

// typedef struct {
//     double frequency;
//     double duration;
// } CPT9091A_Note;

bool cpt9091a_init(DAC_HandleTypeDef* cpt9091a);
void cpt9091a_play_square(DAC_HandleTypeDef* cpt9091a, double volume, double frequency);
void cpt9091a_play_sin(DAC_HandleTypeDef* cpt9091a, double volume, double frequency);
// void cpt9091a_play_notes(DAC_HandleTypeDef* cpt9091a, double volume, CPT9091A_Note* notes, size_t size);
/** Plays "The Iconic Intro Sequence" */
// void cpt9091a_play_pattern_1(DAC_HandleTypeDef* cpt9091a, double volume);
