#include <avr/io.h>
#include <util/delay.h>
#include <Arduino.h>
#include "rosnode.h"
#include "util.h"

#define MOTOR_MIN 50
#define MOTOR_MAX 950
#define ANALOG_READ_MIN 0
#define ANALOG_READ_MAX 1023

static void arduino_init(void)
{
    init();
    #if defined(USBCON)
        USB.attach();
    #endif
    //Serial.begin(57600);
}

void init_pwm(void)
{
    // TCCR = Timer Counter Control Register. It is generally used for
    // prescaling the timer.
    // COM2A1 and COM2B1 means that OC2A and OC2B will be cleared on compare match.
    // WGM21 and WGM20 means that waveform generation mode is set to fast PWM
    TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);

    // by setting WGM22, the fast PWM is now set to mode 7. This means that the
    // counter will reset on the OCR2A value rather than the default 255.
    // The prescaler divisor is set to 1 by setting CS20 only. This means that
    // the counter will run at the clock's full frequency (16 MHz).
    TCCR2B = _BV(WGM22) | _BV(CS20);

    OCR2A = 63;
    
    // This is the duty cycle. It is the last value of the counter our output
    // will remain high for. Can't be greater than OCR2A. A value of 0 means a
    // duty cycle of 1/64 in this case.
    OCR2B = 8;
}

int main(void)
{
    arduino_init();
    node_init();

    int pot = A0;
    pinMode(pot, INPUT);

    // Pin PD3 will be our output (pin 3 on arduino)
    DDRD |= _BV(PD3);
    init_pwm();

    DDRB |= _BV(PB5);
    PORTB |= _BV(PORTB5);

    int32_t pot_value; // a.k.a, current position
    int32_t requested_position;
    while (1) {
        //PORTB &= ~_BV(PORTB5);
        //PORTB ^= _BV(PORTB5);

        pot_value = analogRead(pot);
        pot_value = map(pot_value, ANALOG_READ_MIN, ANALOG_READ_MAX,
                        MOTOR_MIN, MOTOR_MAX);
        node_publish_data(pot_value);

        _delay_ms(2000);

        requested_position = node_get_requested_position();
        if (pot_value > requested_position) {
            PORTB &= ~_BV(PORTB5); // set PHASE to LOW
        } else if (pot_value <= requested_position) {
            PORTB |= _BV(PORTB5); // set PHASE to HIGH
        }
    }
}
