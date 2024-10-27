#ifndef TRANSIMISSION_MANAGER_H
#define TRANSIMISSION_MANAGER_H

#include <stdbool.h>

#define MAX_BURST_SIZE_BYTES 255
#define NUMBER_TELEMETRY_PACKETS 1 // MUST BE NON-ZERO

struct KinematicsData {
    /** The x, y, and z components of the velocity */
    float v_x, v_y, v_z;

    /** The w, x, y, and z components of the attitude */
    float a_w, a_x, a_y, a_z;

    /** The current altitude */
    float altitude;
}; 

/** Stores all data used by transmission.
 * 
 * It is recommended to use the helper methods, as opposed to modifying values in this struct directly.
 * 
 * The number of data substructs in this struct should be equal to NUMBER_TELEMETRY_PACKET
 * 
 * ORDER OF PACKETS: 
 *  0: kinematicsData
 */
struct TransmissionState {
    int nextInQueue;
    bool shouldSendData[NUMBER_TELEMETRY_PACKETS];
    
   
    /** Stores the kinematics data that should be sent */
    struct KinematicsData kinematicsData;
};


inline void queueKinematicsData(struct TransmissionState* state) {
    state->shouldSendData[0] = true;
}

inline struct KinematicsData* getKinematicsDataPtr(struct TransmissionState* state) {
    return &(state->kinematicsData);
}

inline bool shouldSend(struct TransmissionState* state, int index) {
    return state->shouldSendData[index];
}

void sendBurstData(struct TransmissionState* state);

#endif