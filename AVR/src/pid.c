#include <stdlib.h>
#include "pid.h"

void pid_init(pid_state_t *spid, double Kp, double Ki, double Kd,
              double integ_max, double integ_min)
{
    spid->last_pos = 0;
    spid->Ki = Ki;
    spid->Kp = Kp;
    spid->Kd = Kd;
}

static void limit_integrator(pid_state_t *spid)
{
    if (spid->integ_state > spid->integ_max) {
        spid->integ_state = spid->integ_max;
    } else if (spid->integ_state < spid->integ_min) {
        spid->integ_state = spid->integ_min;
    }
}

double update_pid(pid_state_t *spid, int32_t err, int32_t position)
{
    // compute proportional term
    double pterm = spid->Kp*err;

    // compute integral term
    spid->integ_state += err;
    limit_integrator(spid);
    double iterm = spid->Ki*spid->integ_state;

    // compute derivative term
    double dterm = spid->Kd*abs(spid->last_pos - position);

    return pterm + iterm + dterm;
}
