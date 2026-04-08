#ifndef QUATERNION_H
#define QUATERNION_H

typedef struct Quaternion {
    double w, x, y, z;
} quaternion_t;

static const quaternion_t IDENTITY_QUAT = (quaternion_t) {.w = 1, .x = 0, .y = 0, .z = 0};

#define DEG2RAD(x) ((x) * 0.01745329252)
#define RAD2DEG(x) ((x) * 57.2958)


/** Multiplies the quaternion a by the quaternion b */
quaternion_t multiply(quaternion_t* a, quaternion_t* b);

/** Adds to quaternions together */
quaternion_t add(quaternion_t* a, quaternion_t* b);

/** Normalizes the input quaternion inplace */
void normalize_inplace(quaternion_t* a);

/** 
 * Converts a set of angular velocities (in radians) plus a time delta (in seconds) into a delta quaternion 
 */
quaternion_t to_delta_quaternion(double w_x, double w_y, double w_z, double delta_t);

#endif