#include <stdlib.h>
#include "pid.h"
#include "rosnode.h"

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
    int pterm_int = pterm;
    //node_log_info("pterm = %d", pterm_int);

    // compute integral term
    spid->integ_state += err;
    limit_integrator(spid);
    double iterm = spid->Ki*spid->integ_state;
    int iterm_int = iterm;
    //node_log_info("iterm = %d", iterm_int);

    // compute derivative term
    double dterm = spid->Kd*abs(spid->last_pos - position);
    int dterm_int = dterm;
    //node_log_info("dterm = %d", dterm_int);


    int total = pterm + iterm + dterm;
    node_log_info("total = %d", total);
    return pterm_int + iterm_int + dterm_int;
}
