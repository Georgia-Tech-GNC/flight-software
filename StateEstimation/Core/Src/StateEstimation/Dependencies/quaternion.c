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
    double magnitude = sqrt(w_x * w_x + w_y * w_y + w_z * w_z);
    double angle = magnitude * delta_t * 0.5;

    double mult = (delta_t / 2);
    if (angle > __DBL_EPSILON__) {
        mult *= sin(angle) / angle;
    }

    quaternion_t output;
    output.w = cos(angle);
    output.x = w_x * mult;
    output.y = w_y * mult;
    output.z = w_z * mult;

    return output;
}