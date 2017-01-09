#include <avr/io.h>
#include <util/delay.h>

int main(void)
{
    DDRC = 0x0F;

    while (1) {
        /* clockwise rotation */ 
        PORTC = 0b00000101;
        _delay_ms(2000);

        /* counter-clockwise rotation */
        PORTC = 0b00001010;
        _delay_ms(2000);
    }

    return 0;
}
