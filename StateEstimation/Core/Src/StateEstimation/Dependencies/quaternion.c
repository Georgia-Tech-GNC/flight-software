#include "quaternion.h"
#include "math.h"

quaternion_t multiply(quaternion_t* a, quaternion_t* b) {
    quaternion_t output;

    output.w = a->w*b->w - a->x*b->x - a->y*b->y - a->z*b->z;
    output.x = a->w*b->x + a->x*b->w + a->y*b->z - a->z*b->y;
    output.y = a->w*b->y - a->x*b->z + a->y*b->w + a->z*b->x;
    output.z = a->w*b->z + a->x*b->y - a->y*b->x + a->z*b->w;

    return output;
}

quaternion_t add(quaternion_t* a, quaternion_t* b) {
    quaternion_t output;

    output.w = a->w + b->w;
    output.x = a->x + b->x;
    output.y = a->y + b->y;
    output.z = a->z + b->z;

    normalize_inplace(&output);
    return output;
}

void normalize_inplace(quaternion_t* a) {
    double magnitude = sqrt(a->w*a->w + a->x*a->x + a->y*a->y + a->z*a->z);
    if (magnitude < __DBL_EPSILON__) {
        a->w = 1.0;
        a->x = 0.0;
        a->y = 0.0;
        a->z = 0.0;
    } else {
        a->w /= magnitude;
        a->x /= magnitude;
        a->y /= magnitude;
        a->z /= magnitude;      
    }
}

quaternion_t to_delta_quaternion(double w_x, double w_y, double w_z, double delta_t) {
    double magnitude = sqrtf(w_x * w_x + w_y * w_y + w_z * w_z);
    double angle = magnitude * delta_t;

    if (angle < __DBL_EPSILON__) {
        return (quaternion_t) { .w = 1, .x = 0, .y = 0, .z = 0};
    }

    w_x /= magnitude;
    w_y /= magnitude;
    w_z /= magnitude;

    double cos_wt2 = cos(angle / 2);
    double sin_wt2 = sin(angle / 2);

    quaternion_t output;
    output.w = cos_wt2;
    output.x = w_x * sin_wt2;
    output.y = w_y * sin_wt2;
    output.z = w_z * sin_wt2;

    return output;
}