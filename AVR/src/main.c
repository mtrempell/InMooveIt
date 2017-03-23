#include <avr/io.h>
#include <util/delay.h>
#include <Arduino.h>
#include <stdlib.h>
#include "joints.h"
#include "rosnode.h"
#include "util.h"
#include "pid.h"

// TODO: these should be moved to joints.h
#define MOTOR_MIN 200 // FIXME
#define MOTOR_MAX 700
#define ANALOG_READ_MIN 0
#define ANALOG_READ_MAX 1023

#define ERROR_LEEWAY 10


static void arduino_init(void)
{
    init();
    #if defined(USBCON)
        USB.attach();
    #endif
    //Serial.begin(57600);
}

void read_pots(int16_t *current_positions, const struct joint_info *joints,
               size_t num_joints)
{
    for (size_t i = 0; i < num_joints; ++i) {
        current_positions[i] = analogRead(joints[i].pot);
        // TODO: these may vary for every joint
        current_positions[i] = map(current_positions[i], ANALOG_READ_MIN,
                                   ANALOG_READ_MAX, MOTOR_MIN, MOTOR_MAX);
    }
}

void run_pid(const int16_t *requested_positions,
             const int16_t *current_positions, pid_state_t *pid_states,
             struct joint_info *joints, int num_joints)
{
    int16_t position_err;
    double motor_speed;
    for (size_t i = 0; i < num_joints; ++i) {
        position_err = abs(requested_positions[i] - current_positions[i]);

        analogWrite(joints[i].active_pwm_pin, 0); // shut down current PWM signal
        if (current_positions[i] > requested_positions[i]) {
            joint_set_pwm_reverse(&joints[i]);
        } else {
            joint_set_pwm_forward(&joints[i]); 
        }

        if (position_err > ERROR_LEEWAY) {
            motor_speed = update_pid(&pid_states[i], position_err,
                                     current_positions[i]);
            analogWrite(joints[i].active_pwm_pin, motor_speed);
        }

    }
}

int main(void)
{
    arduino_init();
    node_init();

    // initialize inputs/outputs FIXME this needs to be generalized to more joints
    struct joint_info joints[NUM_OF_JOINTS];
    joints[0].pwm_pins[0] = 5; joints[0].pwm_pins[1] = 6;
    joints[0].pot = A0;
    for (size_t i = 0; i < NUM_OF_JOINTS; ++i) {
        pinMode(joints[i].pot, INPUT);
        pinMode(joints[i].pwm_pins[0], OUTPUT);
        pinMode(joints[i].pwm_pins[1], OUTPUT);
        joints[i].active_pwm_pin = 0;
        // TODO: do the rest and maybe move this to a separate function
    }

    pid_state_t pid_states[NUM_OF_JOINTS];
    for (size_t i = 0; i < NUM_OF_JOINTS; ++i) {
        pid_init(&pid_states[i], 1, 100, 0.01, 0, 3000);
    }

    int16_t current_positions[NUM_OF_JOINTS];
    const int16_t *requested_positions;
    while (1) {
        read_pots(current_positions, joints, NUM_OF_JOINTS);
        node_publish_data(current_positions);

        requested_positions = node_get_requested_positions();
        run_pid(requested_positions, current_positions, pid_states, joints,
                NUM_OF_JOINTS);
    }
}
