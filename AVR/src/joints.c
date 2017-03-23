#include <stdlib.h>
#include "joints.h"

void joint_set_pwm_forward(struct joint_info *joint)
{
    joint->active_pwm_pin = joint->pwm_pins[0];
}

void joint_set_pwm_reverse(struct joint_info *joint)
{
    joint->active_pwm_pin = joint->pwm_pins[1];
}