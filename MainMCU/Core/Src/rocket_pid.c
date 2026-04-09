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
quaternion_t quaternion_t_normalize(quaternion_t* q) {
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

/* -----------------------------------------------------------------------
 * pid_get_error
 *
 * replaces: function err = getError(obj, rocketStateEstimate, currentTime)
 *
 * computes the rotation-vector attitude error between the rocket's
 * current quaternion_ternion and the reference quaternion_ternion at the nearest time.
 * ----------------------------------------------------------------------- */
void pid_get_error(const PIDController *ctrl,
                   const double state[14], double currentTime,
                   double err_out[3])
{
    /* q_cur = Utils.normalizequaternion_t(rocketStateEstimate(8:11))
     * MATLAB 1-indexed 8:11 -> C 0-indexed 7:10 */
    quaternion_t q_cur = { state[7], state[8], state[9], state[10] };
    q_cur = quaternion_t_normalize(&q_cur);

    /* [~, idx] = min(abs(time_vec - currentTime)) */
    int idx = 0; // ref_find_closest(ctrl->ref, currentTime);

    /* q_ref = Utils.normalizequaternion_t(obj.referenceAttitudes(2:5, idx)) */
    quaternion_t q_ref = (quaternion_t) { .w = 1.0, .x = 0.0, .y = 0.0, .z = 0.0};

    /* q_cur_inv = [q_cur(1); -q_cur(2); -q_cur(3); -q_cur(4)] */
    quaternion_t q_cur_inv = quaternion_t_conjugate(&q_cur);

    /* q_err = Utils.quaternion_tMult(q_cur_inv, q_ref) */
    quaternion_t q_err = quaternion_t_multiply(&q_cur_inv, &q_ref);

    /* if q_err(1) < 0; q_err = -q_err; end */
    if (q_err.w < 0.0) {
        q_err.w = -q_err.w;
        q_err.x = -q_err.x;
        q_err.y = -q_err.y;
        q_err.z = -q_err.z;
    }

    /* vec_norm = norm(q_err(2:4)) */
    double vec_norm = sqrt(q_err.x*q_err.x + q_err.y*q_err.y + q_err.z*q_err.z);

    /* if vec_norm < 1e-20; err = zeros(3,1) */
    if (vec_norm < __DBL_EPSILON__) {
        err_out[0] = 0.0;
        err_out[1] = 0.0;
        err_out[2] = 0.0;
    } else {
        /* theta = 2 * atan2(vec_norm, q_err(1)) */
        double theta = 2.0 * atan2(vec_norm, q_err.w);
        /* n_hat = q_err(2:4) / vec_norm */
        /* err   = theta * n_hat */
        err_out[0] = theta * (q_err.x / vec_norm);
        err_out[1] = theta * (q_err.y / vec_norm);
        err_out[2] = theta * (q_err.z / vec_norm);
    }
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
void getControl(PIDController *ctrl,
                     const double state[14],
                     double currentTime,
                     double rocketControl_out[3])
{
    /* dt = currentTime - obj.lastTime */
    double dt = currentTime - ctrl->lastTime;
    /* if dt <= 0; dt = 1e-4; end */
    if (dt <= 0.0) dt = 1e-4;

    /* err = obj.getError(state, currentTime) */
    double err[3];
    pid_get_error(ctrl, state, currentTime, err);

    /* obj.integralError = obj.integralError + err * dt */
    for (int i = 0; i < 3; i++) {
        ctrl->integralError[i] += err[i] * dt;
    }

    /* omega = state(12:14)  — MATLAB 1-indexed -> C 0-indexed 11:13 */
    double omega_raw[3] = { state[11], state[12], state[13] };

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
        controlRad[i] = ctrl->Kp * err[i]
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
}
