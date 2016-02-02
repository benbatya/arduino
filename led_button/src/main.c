#include <avr/io.h>
//#include "Arduino.h"
#include <util/delay.h>

uint8_t ReadButton(void);

#define BUTTON_PIN (1<<PD2)
#define LED_PIN (1<<PB5)

int main(void)
{
    DDRD &= ~BUTTON_PIN; // Configure PORTD pin 2 as input
    PORTD |= BUTTON_PIN; // Activate pull-ups in PORTD pin 2
    DDRB |= LED_PIN;  // Configure arduino pin 13 (led) to output

    while (1) 
    {
        if (ReadButton()) 
        {
            PORTB |= LED_PIN; // toggle the led
        } else
        {
            PORTB &= ~LED_PIN;
        }
        _delay_ms(250);
    }
}

uint8_t ReadButton(void)
{
    if ((PIND & BUTTON_PIN) == 0) // Button was pressed
    {
        _delay_ms(25);  // Wait to read again because of the debounce
    }
    if ((PIND & BUTTON_PIN) == 0) // Verify that the button was pushed
    {
        return 1;
    } else
    {
        return 0;
    }
}

