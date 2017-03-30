#ifndef JOINTS_H_GUARD
#define JOINTS_H_GUARD

#include <stdint.h>

#define NUM_OF_JOINTS 1

struct joint_info {
    int pwm_pins[2]; // forward pin == pwm_pins[0], reverse == pwm_pins[1]
    int active_pwm_pin; // identifies which PWM pin is currently used
    int pot; // pot pin number
    int max_pos; // maximum pot value
    int min_pos; // minimum pot value
    int err_leeway;
    int default_position;

    // maybe add requested positions and shit here??
};

void joint_set_pwm_forward(struct joint_info *joint);
void joint_set_pwm_reverse(struct joint_info *joint);

void get_joint_info(struct joint_info *joints);

// maybe define the max and min positions of these joints

#endif /* JOINTS_H_GUARD */
