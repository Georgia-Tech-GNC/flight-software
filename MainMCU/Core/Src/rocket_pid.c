/* Author(s): Liam Kerr, Malachi Muhic
 * rocket_pid.c
 *
 * TODO:
 *      Further questions: Anti windup gain constant
 * 
 * Attitude Controller for a Rocket
 * Ported from MATLAB to C with one modification:
 *   - Added first-order EMA low-pass filter on the derivative (omega) term
 *
 * Original MATLAB interface preserved:
 *   - getControl(obj, state, currentCumVelocity, currentTime) -> rocketControl
 *   - getError(obj, rocketStateEstimate, currentTime) -> err
 *
 * State vector convention (14 elements, 0-indexed in C):
 *   [0..2]   position (x, y, z)           — unused by this controller
 *   [3..6]   velocity / other              — unused by this controller
 *   [7..10]  quaternion_ternion (w, x, y, z)       — MATLAB indices 8:11
 *   [11..13] angular velocity (rad/s)      — MATLAB indices 12:14
 *
 * quaternion_ternion convention: scalar-first [w, x, y, z]
 *
 * Compile:  gcc -o rocket_pid rocket_pid.c -lm
 * Run:      ./rocket_pid
 */

#include "rocket_pid.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// normalize quaternion_ternion
quaternion_t quat_normalize(quaternion_t* q) {
    double mag = sqrt(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);
    if (mag < 1e-30) {
        quaternion_t identity = {1.0, 0.0, 0.0, 0.0};
        return identity;
    }
    quaternion_t out = { q->w/mag, q->x/mag, q->y/mag, q->z/mag };
    return out;
}

// conjugate: to represent opposite rotation
quaternion_t quaternion_t_conjugate(quaternion_t* q) {
    quaternion_t out = { q->w, -q->x, -q->y, -q->z };
    return out;
}

// compares where rocket is to where the rocket should be
quaternion_t quaternion_t_multiply(quaternion_t* a, quaternion_t* b) {
    quaternion_t out;
    out.w = a->w*b->w - a->x*b->x - a->y*b->y - a->z*b->z;
    out.x = a->w*b->x + a->x*b->w + a->y*b->z - a->z*b->y;
    out.y = a->w*b->y - a->x*b->z + a->y*b->w + a->z*b->x;
    out.z = a->w*b->z + a->x*b->y - a->y*b->x + a->z*b->w;
    return out;
}

vector3_t vector_cross(const vector3_t* a, const vector3_t* b) {
    vector3_t out;
    out.x = a->y * b->z - b->y * a->z;
    out.y = -a->x * b->z + b->x * a->z;
    out.z = a->x * b->y - b->x * a->y;
    return out;
}

double vector_dot(const vector3_t* a, const vector3_t* b) {
    return a->x * b->x + a->y * b->y + a->z * b->z;
}

double vector_mag(const vector3_t* a) {
    return sqrt(a->x * a->x + a->y * a->y + a->z * a->z);
}

vector3_t vector_add(const vector3_t* a, const vector3_t* b) {
    return (vector3_t) { .x = a->x + b->x, .y = a->y + b->y, .z = a->z + b->z}; 
}

vector3_t scalar_add(const vector3_t* a, double b) {
    return (vector3_t) { .x = a->x + b, .y = a->y + b, .z = a->z + b};
}

vector3_t scalar_prod(const vector3_t* a, double b) {
    return (vector3_t) { .x = a->x * b, .y = a->y * b, .z = a->z * b};
}

vector3_t quaternion_rotate(const vector3_t* vec, const quaternion_t* quat) {
    double quat_w = quat->w;
    vector3_t quat_vec = (vector3_t) {.x = quat->x, .y = quat->y, .z = quat->z};

    vector3_t t = vector_cross(&quat_vec, vec);
    t = scalar_prod(&t, 2);

    vector3_t i2 = vector_cross(&quat_vec, &t);
    vector3_t wt = scalar_prod(&t, quat_w);

    vector3_t i3 = vector_add(vec, &wt);
    return vector_add(&i3, &i2);
}

#define DEG2RAD(x) ((x) * M_PI / 180.0)

const double PHI = -M_PI / 4;

// linear scan to find closest waypoint in time for which reference orientation to aim for
int ref_find_closest(const RefTrajectory *ref, double t) {
    int best = 0;
    double best_diff = fabs(ref->time[0] - t);
    for (int i = 1; i < ref->count; i++) {
        double diff = fabs(ref->time[i] - t);
        if (diff < best_diff) {
            best_diff = diff;
            best = i;
        }
    }
    return best;
}

/*PID Controller*/
/* Initialize controller */
void pid_init(PIDController *ctrl,
              const double Kp, const double Ki, const double Kd,
              double tau)
{
    for (int i = 0; i < 3; i++) {
        
        ctrl->integralError[i] = 0.0;
        ctrl->filteredOmega[i] = 0.0;
    }
    ctrl->Kp = Kp;
    ctrl->Ki = Ki;
    ctrl->Kd = Kd;
    ctrl->lastTime          = 0.0;
    ctrl->tau               = tau;
    ctrl->alpha             = 0.0;
    ctrl->filterInitialized = 0;
}

const quaternion_t Q_REF = (quaternion_t) { .w = 1.0, .x = 0.0, .y = 0.0, .z = 0.0};

/* -----------------------------------------------------------------------
 * pid_get_error
 *
 * replaces: function err = getError(obj, rocketStateEstimate, currentTime)
 *
 * computes the rotation-vector attitude error between the rocket's
 * current quaternion_ternion and the reference quaternion_ternion at the nearest time.
 * ----------------------------------------------------------------------- */
double pid_get_error(const PIDController *ctrl,
                   const double state[14], double currentTime,
                   double err_out[3])
{
    /* q_cur = Utils.normalizequaternion_t(rocketStateEstimate(8:11))
     * MATLAB 1-indexed 8:11 -> C 0-indexed 7:10 */
    quaternion_t q_cur = { state[7], state[8], state[9], state[10] };
    //q_imu = quat_normalize(&q_imu);
    //quaternion_t q_cur = quaternion_t_multiply(&Q_CORRECT,&q_imu);
    q_cur = quat_normalize(&q_cur);

    /* q_cur_inv = [q_cur(1); -q_cur(2); -q_cur(3); -q_cur(4)] */
    quaternion_t q_cur_inv = quaternion_t_conjugate(&q_cur);
    quaternion_t q_ref_inv = quaternion_t_conjugate(&Q_REF);

    vector3_t vb = (vector3_t) { .x = 1, .y = 0, .z = 0};
    vector3_t v_curr = quaternion_rotate(&vb, &q_cur);
    vector3_t v_desired = quaternion_rotate(&vb, &Q_REF);

    double theta = acos(vector_dot(&v_curr, &v_desired) / (vector_mag(&v_curr) * vector_mag(&v_desired)));

    double d = vector_dot(&v_curr, &v_desired);
    vector3_t a = vector_cross(&v_curr, &v_desired);

    quaternion_t error = (quaternion_t) {
        .w = 1 + d,
        .x = a.x, .y = a.y, .z = a.z
    };
    error = quat_normalize(&error);
    vector3_t error_vec = (vector3_t) {.x = error.x, .y = error.y, .z = error.z};

    vector3_t error_body = quaternion_rotate(&error_vec, &q_cur_inv);
    error_body.y *= 2;
    error_body.z *= 2;

    err_out[0] = error_body.x * 2;
    err_out[1] = cos(PHI)*error_body.y - sin(PHI)*error_body.z;
    err_out[2] = sin(PHI)*error_body.y + cos(PHI)*error_body.z;

    return theta;
}

/* -----------------------------------------------------------------------
 * getControl
 *
 * replaces: function rocketControl = getControl(obj, state,
 *               currentCumVelocity, currentTime)
 *
 * MODIFICATION: omega is now passed through a first-order EMA low-pass
 * filter before being used in the Kd term. All other logic is identical
 * to the original MATLAB.
 * ----------------------------------------------------------------------- */
double getControl(PIDController *ctrl,
                     const double state[14],
                     double currentTime,
                     double rocketControl_out[3], double err_out[3])
{
    /* dt = currentTime - obj.lastTime */
    double dt = currentTime - ctrl->lastTime;
    /* if dt <= 0; dt = 1e-4; end */
    if (dt <= 0.0) dt = 1e-4;

    /* err = obj.getError(state, currentTime) */
    double error_angle = pid_get_error(ctrl, state, currentTime, err_out);

    /* obj.integralError = obj.integralError + err * dt */
    for (int i = 0; i < 3; i++) {
        ctrl->integralError[i] += err_out[i] * dt;
    }

    /* omega = state(12:14)  — MATLAB 1-indexed -> C 0-indexed 11:13 */
    vector3_t omega_body = { state[11], state[12], state[13] };
    /* Rotate angular velocity into body/gimbal frame*/
    //vector3_t omega_body = quaternion_rotate(&omega_imu, &Q_CORRECT);
    double omega_raw[3] = { omega_body.x, 0, 0 };
    omega_raw[1] = -cos(PHI)*omega_body.y + sin(PHI)*omega_body.z;
    omega_raw[2] = -sin(PHI)*omega_body.y - cos(PHI)*omega_body.z;

    /* ---------------------------------------------------------------
     * NEW: low-pass filter (first-order EMA) on the derivative term
     *
     * this replaces the direct use of omega:
     *   MATLAB original:  Kd .* omega
     *   C:       Kd .* filteredOmega
     *   alpha = dt / (tau + dt)
     *   filtered = alpha * raw + (1 - alpha) * filtered_prev
     *
     * on the very first call, initialize the filter to the raw value
     * to avoid a startup transient from zero.
     * --------------------------------------------------------------- */
    ctrl->alpha = dt / (ctrl->tau + dt);

    if (!ctrl->filterInitialized) {
        for (int i = 0; i < 3; i++) {
            ctrl->filteredOmega[i] = omega_raw[i];
        }
        ctrl->filterInitialized = 1;
    } else {
        for (int i = 0; i < 3; i++) {
            ctrl->filteredOmega[i] = ctrl->alpha * omega_raw[i]
                                   + (1.0 - ctrl->alpha) * ctrl->filteredOmega[i];
        }
    }

    /* rocketControlRad = obj.Kp .* err + obj.Ki .* obj.integralError + obj.Kd .* omega
     * omega replaced with filteredOmega (the only change from original) */
    double controlRad[3];
    for (int i = 0; i < 3; i++) {
        controlRad[i] = ctrl->Kp * err_out[i]
                       + ctrl->Ki * ctrl->integralError[i]
                       + ctrl->Kd * ctrl->filteredOmega[i];
    }

    /* debug print (first 12 seconds)
     * extended from original to also show raw vs filtered omega */
    if (currentTime < 12.0) {
        /*
        printf("  t=%.2f  err=[%.4f %.4f %.4f]  omega_raw=[%.4f %.4f %.4f]  "
               "omega_filt=[%.4f %.4f %.4f]  u=[%.4f %.4f %.4f]\n",
               currentTime,
               err[0], err[1], err[2],
               omega_raw[0], omega_raw[1], omega_raw[2],
               ctrl->filteredOmega[0], ctrl->filteredOmega[1], ctrl->filteredOmega[2],
               controlRad[0], controlRad[1], controlRad[2]);
        */
    }

    /* obj.lastTime = currentTime */
    ctrl->lastTime = currentTime;

    /* rocketControl = rad2deg(rocketControlRad) */
    for (int i = 0; i < 3; i++) {
        rocketControl_out[i] = controlRad[i]; // * (180.0 / M_PI);
    }

    return error_angle;
}
