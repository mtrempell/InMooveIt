#include <stdlib.h>
#include <Arduino.h>
#include "joints.h"
#include "rosnode.h"


void joint_set_pwm_forward(struct joint_info *joint)
{
    joint->active_pwm_pin = joint->pwm_pins[0];
}

void joint_set_pwm_reverse(struct joint_info *joint)
{
    joint->active_pwm_pin = joint->pwm_pins[1];
}


#define ERR_LEEWAY_DEFAULT 10
#define MIN_POS_DEFAULT 300
#define MAX_POS_DEFAULT 500
#define DEFAULT_KP 1
#define DEFAULT_KI 100
#define DEFAULT_KD 0.01

void set_elbow_joint_info(struct joint_info *joint)
{
    joint->pwm_pins[0] = 5;
    joint->pwm_pins[1] = 6;
    joint->pot = A0;
    joint->min_pos = 430;
    joint->max_pos = 855;
    joint->name = "elbow";
}

void set_shoulder_rotate_joint_info(struct joint_info *joint)
{
    //joint->pwm_pins[0]
    joint->min_pos = 30;
    joint->max_pos = 885;

}

void set_shoulder_vertical_joint_info(struct joint_info *joint)
{
    joint->min_pos = 150;
    joint->max_pos;
}

// returns pre-defined joint infos
// make SURE that these match NUM_OF_JOINTS
void get_joint_info(struct joint_info *joints)
{

    // set default values for all joints
    for (size_t i = 0; i < NUM_OF_JOINTS; ++i) {
        joints[i].min_pos = MIN_POS_DEFAULT;
        joints[i].max_pos = MAX_POS_DEFAULT;
        joints[i].err_leeway = ERR_LEEWAY_DEFAULT;

        joints[i].kp = DEFAULT_KP;
        joints[i].kd = DEFAULT_KD;
        joints[i].ki = DEFAULT_KI;
    }

    // joint definitions -- MUST MATCH NUM_OF_JOINTS
    set_elbow_joint_info(&joints[0]);


    // set all active pins to FORWARD
    for (size_t i = 0; i < NUM_OF_JOINTS; i++) {
        joints[i].active_pwm_pin = joints[i].pwm_pins[0];
    }

    for (size_t i = 0; i < NUM_OF_JOINTS; ++i) {
        node_log_info("'%s' initialized as joint %d", joints[i].name, i);
    }
}
