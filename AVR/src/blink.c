#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include "bitops.h"

int main(void)
{
    uint16_t toggle_count = 31999; // 1/500'th of a second
    /* set pin 5 of PORTB for output */
    DDRB |= _BV(DDB5);

    // setup timer
    TCCR1B |= _BV(CS10);

    while (1) {
        
        if (TCNT1 >= toggle_count) {
            PORTB ^= _BV(PORTB5); // toggle led on port 5

            TCNT1 = 0;
        }
    }

    return 0;
}


/* NOTES:
 * 
 * - _BV == bit value e.g. _BV(3) --> 1 << 3 --> 0x08
 */
