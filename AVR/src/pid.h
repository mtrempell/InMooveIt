#ifndef PID_GUARD_H
#define PID_GUARD_H
#include <stdint.h>

typedef struct {
    double last_pos;

    // integrator values
    double integ_state;
    double integ_max;
    double integ_min;

    // gain constants
    double Ki;
    double Kp;
    double Kd;
} pid_state_t;

void pid_init(pid_state_t *spid, double Kp, double Ki, double Kd,
              double integ_max, double integ_min);
double update_pid(pid_state_t *spid, int32_t err, int32_t position);

#endif /* PID_GUARD_H */
