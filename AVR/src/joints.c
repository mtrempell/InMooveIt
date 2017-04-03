#include <stdlib.h>
#include <Arduino.h>
#include "util.h"
#include "joints.h"
#include "rosnode.h"


void joint_set_pwm_forward(struct joint_info *joint)
{
    if (strcmp(joint->name, "wrist") == 0) {
        digitalWrite(joint->phase_pin, HIGH);
    } else if (joint->active_pwm_pin == joint->pwm_pins[1]) {
        // shut down reverse PWM signal
        analogWrite(joint->active_pwm_pin, 0);
        joint->active_pwm_pin = joint->pwm_pins[0];
    }

    // else do nothing
}

void joint_set_pwm_reverse(struct joint_info *joint)
{
    if (strcmp(joint->name, "wrist") == 0) {
        digitalWrite(joint->phase_pin, LOW);
    } else if (joint->active_pwm_pin == joint->pwm_pins[0]) {
        // shut down forward PWM signal
        analogWrite(joint->active_pwm_pin, 0);
        joint->active_pwm_pin = joint->pwm_pins[1];
    }

    // else do nothing
}


#define ERR_LEEWAY_DEFAULT 10
#define MIN_POS_DEFAULT 300
#define MAX_POS_DEFAULT 500
#define DEFAULT_KP 1
#define DEFAULT_KI 100
#define DEFAULT_KD 0.01

static void set_elbow_joint_info(struct joint_info *joint)
{
    joint->pwm_pins[0] = 8;
    joint->pwm_pins[1] = 9;
    joint->pot = A3;
    joint->min_pos = 170;
    joint->max_pos = 570;
    strcpy(joint->name, "elbow");

    joint->default_position = 400;
}

// shoulder joint #3
static void set_shoulder_rotate_joint_info(struct joint_info *joint)
{
    joint->pwm_pins[0] = 12;
    joint->pwm_pins[1] = 13;
    joint->pot = A0;
    joint->min_pos = 140; // around 180 deg of rotation
    joint->max_pos = 915;
    strcpy(joint->name, "shoulder_rotate");

    joint->default_position = 500;
}


// shoulder joint #2
static void set_shoulder_horizontal_joint_info(struct joint_info *joint)
{
    joint->pwm_pins[0] = 10;
    joint->pwm_pins[1] = 11;
    joint->pot = A1;
    joint->min_pos = 40;
    joint->max_pos = 655; // around 180 deg of rotation
    strcpy(joint->name, "shoulder_horizontal");

    joint->default_position = 425;
}

static void set_shoulder_vertical_joint_info(struct joint_info *joint)
{
    joint->pwm_pins[0] = 5;
    joint->pwm_pins[1] = 6;
    joint->pot = A2;
    joint->min_pos = 660;
    joint->max_pos = 780; // around 180 deg of rotation
    strcpy(joint->name, "shoulder_vertical");

    joint->default_position = 425;
}

static void set_wrist_joint_info(struct  joint_info *joint)
{
    joint->pwm_pins[0] = 3;
    joint->pwm_pins[1] = 4;
    joint->pot = A4;
    joint->min_pos = 50;
    joint->max_pos = 910;
    strcpy(joint->name, "wrist");
    joint->phase_pin = 51;   
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
    //set_shoulder_vertical_joint_info(&joints[0]);
    set_wrist_joint_info(&joints[0]);
    set_elbow_joint_info(&joints[1]);
    set_shoulder_horizontal_joint_info(&joints[2]);
    set_shoulder_rotate_joint_info(&joints[3]);


    // set all active pins to FORWARD
    for (size_t i = 0; i < NUM_OF_JOINTS; i++) {
        joints[i].active_pwm_pin = joints[i].pwm_pins[0];
    }

    for (size_t i = 0; i < NUM_OF_JOINTS; ++i) {
        node_log_info("'%s' initialized as joint %d", joints[i].name, i);
    }
}
