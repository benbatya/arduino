#ifndef MORSE_H
#define MORSE_H

class Morse
{
public:
    Morse(int pin);
    void dot();
    void dash();

private:
    int _pin;
};

#endif // __MORSE_H__