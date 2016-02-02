#include <avr/io.h>
#include <util/delay.h>

uint8_t ReadButton(void);

#define LED_PIN (1<<PB5)

int main(void)
{
    DDRB |= LED_PIN;  // Configure arduino pin 13 (led) to output

    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

    ADMUX |= (1 << REFS0);
    ADMUX |= (1 << ADLAR);

    ADCSRA |= (1 << ADFR);

    ADCSRA |= (1 << ADEN);
    ADCSRA |= (1 << ADSC);

    for (;;) 
    {

        if (ADCH < 128) 
        {
            PORTB |= LED_PIN;
        } else
        {
            PORTB &= ~LED_PIN;
        }
    }
}


