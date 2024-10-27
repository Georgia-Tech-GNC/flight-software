#include "transmission_manager.h"

// BROKEN
void sendBurstData(struct TransmissionState* state) {
    int sent_bytes = 0;

    // Check array bounds for safety
    if (state->nextInQueue < 0 || state->nextInQueue >= NUMBER_TELEMETRY_PACKETS) {
        state->nextInQueue = 0;
    }

    for (int number_checked_packets = 0; number_checked_packets < NUMBER_TELEMETRY_PACKETS; number_checked_packets++) {
        //if (!shouldSend(state, state->nextInQueue)) {
        //    continue;
        //}

        // TODO: Send this message and increment sent_bytes

        state->nextInQueue++;
        if (state->nextInQueue >= NUMBER_TELEMETRY_PACKETS) {
            state->nextInQueue = 0;
        }

        // Reached max burst send capacity
        if (sent_bytes >= MAX_BURST_SIZE_BYTES) {
            return;
        }
    }
}