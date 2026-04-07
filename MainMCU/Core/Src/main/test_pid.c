/*
 * test_pid.c
 *
 * Interactive test harness for the rocket PID attitude controller.
 * This is for standalone testing only — not part of the flight code.
 *
 * Compile:  gcc -o test_pid rocket_pid.c test_pid.c -lm
 * Run:      ./test_pid
 */

#include "rocket_pid.h"

#include <stdio.h>
#include <stdlib.h>

void print_separator(void) {
    printf("--------------------------------------------------------------\n");
}

void read_vec3(const char *prompt, double out[3]) {
    printf("  %s (3 values, space-separated): ", prompt);
    if (scanf("%lf %lf %lf", &out[0], &out[1], &out[2]) != 3) {
        printf("  Invalid input, using 0 0 0\n");
        out[0] = out[1] = out[2] = 0.0;
        int c; while ((c = getchar()) != '\n' && c != EOF);
    }
}

void read_quat(const char *prompt, Quat *q) {
    printf("  %s (w x y z, space-separated): ", prompt);
    if (scanf("%lf %lf %lf %lf", &q->w, &q->x, &q->y, &q->z) != 4) {
        printf("  Invalid input, using identity [1 0 0 0]\n");
        q->w = 1.0; q->x = 0.0; q->y = 0.0; q->z = 0.0;
        int c; while ((c = getchar()) != '\n' && c != EOF);
    }
}

int main(void) {
    printf("==========================================================\n");
    printf("  Rocket PID Attitude Controller — Interactive Test\n");
    printf("==========================================================\n\n");

    /* --- Set up gains --- */
    double Kp[3], Ki[3], Kd[3];
    printf("[1] PID Gains\n");
    read_vec3("Kp (proportional)", Kp);
    read_vec3("Ki (integral)",     Ki);
    read_vec3("Kd (derivative)",   Kd);
    print_separator();

    /* --- Filter time constant --- */
    double tau;
    printf("[2] Low-Pass Filter\n");
    printf("  Time constant tau (seconds, e.g. 0.05): ");
    if (scanf("%lf", &tau) != 1 || tau < 0.0) {
        printf("  Invalid, using tau = 0.05\n");
        tau = 0.05;
        int c; while ((c = getchar()) != '\n' && c != EOF);
    }
    print_separator();

    /* --- Reference trajectory --- */
    printf("[3] Reference Attitude Trajectory\n");
    RefTrajectory ref;
    printf("  How many reference points? ");
    if (scanf("%d", &ref.count) != 1 || ref.count < 1) {
        printf("  Using 1 default point: t=0, q=[1 0 0 0]\n");
        ref.count   = 1;
        ref.time[0] = 0.0;
        ref.quat[0] = (Quat){1.0, 0.0, 0.0, 0.0};
    } else {
        if (ref.count > MAX_REF_POINTS) ref.count = MAX_REF_POINTS;
        for (int i = 0; i < ref.count; i++) {
            printf("  Point %d:\n", i);
            printf("    Time: ");
            scanf("%lf", &ref.time[i]);
            read_quat("   Quaternion", &ref.quat[i]);
        }
    }
    print_separator();

    /* --- Initialize controller --- */
    PIDController ctrl;
    pid_init(&ctrl, Kp, Ki, Kd, tau, &ref);

    /* --- Interactive control loop --- */
    printf("[4] Control Loop — enter state vectors one at a time\n");
    printf("    State vector layout (14 elements, 0-indexed):\n");
    printf("      [0..2]  = position (x, y, z)         — not used by controller\n");
    printf("      [3..6]  = velocity or other           — not used by controller\n");
    printf("      [7..10] = quaternion (w, x, y, z)\n");
    printf("      [11..13]= angular velocity (rad/s)\n");
    printf("    Also provide: cumulative velocity and current time.\n");
    printf("    Type 'q' to quit.\n\n");

    while (1) {
        print_separator();
        printf("  Enter current time (or 'q' to quit): ");
        char buf[64];
        if (scanf("%63s", buf) != 1) break;
        if (buf[0] == 'q' || buf[0] == 'Q') break;
        double currentTime = atof(buf);

        double cumVel;
        printf("  Cumulative velocity: ");
        scanf("%lf", &cumVel);

        double state[14];
        printf("  Position (x y z):           ");
        scanf("%lf %lf %lf", &state[0], &state[1], &state[2]);

        printf("  State[3..6] (4 values):     ");
        scanf("%lf %lf %lf %lf", &state[3], &state[4], &state[5], &state[6]);

        printf("  Quaternion (w x y z):       ");
        scanf("%lf %lf %lf %lf", &state[7], &state[8], &state[9], &state[10]);

        printf("  Angular velocity (rad/s):   ");
        scanf("%lf %lf %lf", &state[11], &state[12], &state[13]);

        /* Compute control */
        double control[3];
        getControl(&ctrl, state, cumVel, currentTime, control);

        printf("\n  >>> Actuator command (degrees): [%.4f  %.4f  %.4f]\n",
               control[0], control[1], control[2]);
        printf("  >>> Integral error state:       [%.4f  %.4f  %.4f]\n",
               ctrl.integralError[0], ctrl.integralError[1], ctrl.integralError[2]);
        printf("  >>> Filtered omega:             [%.4f  %.4f  %.4f]\n\n",
               ctrl.filteredOmega[0], ctrl.filteredOmega[1], ctrl.filteredOmega[2]);
    }

    printf("\nDone.\n");
    return 0;
}
