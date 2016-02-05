#include "Morse.h"


int main(void)
{
    
    Morse morse(13);

    while(true)
    {
        morse.dot(); morse.dot(); morse.dot();
        morse.dash(); morse.dash(); morse.dash();
        morse.dot(); morse.dot(); morse.dot();
        delay(3000);
    }
}

