/*
 * rocket_pid.h
 *
 * PID Attitude Controller for a Rocket
 * Ported from MATLAB (Legacy/legacy-control-pid.m) to C with one modification:
 *   - Added first-order EMA low-pass filter on the derivative (omega) term
 *
 * Author(s) Malachi Muhic, Liam Kerr
 */

#ifndef ROCKET_PID_H
#define ROCKET_PID_H

// Convention: q = [w, x, y, z] (scalar first)

// Quaternion (Quat): 4-number representation of 3D rotation 
typedef struct {
    double w, x, y, z;
} Quat;

// Normalize Quat: Floating-point drift, sensor noise, or interpolation can push the magnitude slightly off
Quat quat_normalize(Quat q);

// Conjugate: to represent opposite rotation
Quat quat_conjugate(Quat q);

// compares where rocket is to where the rocket should be
Quat quat_multiply(Quat a, Quat b);

#define MAX_REF_POINTS 1024 // maximum size of the reference trajectory table

// Reference Trajectory
// @ time t, rocket should be oriented as quat[i]
    // Could build off of this in the future to interpolate between the two nearest points instead of...
    // snapping to the closest one (good enough if reference is dense)
typedef struct {
    int      count;                     // How many waypoints are filled in
    double   time[MAX_REF_POINTS];      // Timestamps in seconds from launch
    Quat     quat[MAX_REF_POINTS];      // Parallel list of quaternions, one per timestamp
} RefTrajectory;


// linear scan to find closest waypoint in time for which reference orientation to aim for
int ref_find_closest(const RefTrajectory *ref, double t);

/* ======================================================================
 * PID Controller
 *
 * Three-axis PID with per-axis gains.  The derivative (Kd) channel
 * passes angular velocity through a first-order EMA low-pass filter
 * before use — the only change from the original MATLAB.
 * ====================================================================== */

typedef struct {
    // PID gains, indexed per body axis: [0]=roll, [1]=pitch, [2]=yaw 
    double Kp[3];   /* Proportional gain */
    double Ki[3];   /* Integral gain     */
    double Kd[3];   /* Derivative gain   */

    // Persistent controller state, updated every call to pid_get_control 
    double integralError[3];  // Accumulated error per axis (rad*s) 
    double lastTime;          // Timestamp of previous call, used to compute dt 

    /* Derivative-term low-pass filter (first-order EMA on measured omega).
     * Not present in the original MATLAB — added during the C port to
     * suppress gyro noise before it gets multiplied by Kd. */
    double filteredOmega[3];  // Filter state, one per axis (rad/s) 
    double alpha;             // Smoothing factor, recomputed each tick as dt/(tau+dt).
    double tau;               // Filter time constant (seconds). Larger tau = smoother
    int    filterInitialized; 

    const RefTrajectory *ref;
} PIDController;

void pid_init(PIDController *ctrl,
              const double Kp[3], const double Ki[3], const double Kd[3],
              double tau, const RefTrajectory *ref);

// Compute the rotation-vector attitude error 
void pid_get_error(const PIDController *ctrl,
                   const double state[14], double currentTime,
                   double err_out[3]);

// Flight control loop for every tick (t)
/* Compute the actuator command (degrees, per axis).
 * state            — 14-element state vector
 * currentCumVelocity — passed for interface compatibility (unused)
 * currentTime      — simulation clock (seconds)
 * rocketControl_out — output: [roll, pitch, yaw] command in degrees */
void getControl(PIDController *ctrl,
                     const double state[14], double currentCumVelocity,
                     double currentTime,
                     double rocketControl_out[3]);

#endif 
