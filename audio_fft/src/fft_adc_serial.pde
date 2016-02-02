/*
fft_adc_serial.pde
guest openmusiclabs.com 7.7.14
example sketch for testing the fft library.
it takes in data on ADC0 (Analog0) and processes them
with the fft. the data is sent out over the serial
port at 115.2kb.
*/

#define LOG_OUT 1 // use the log output function
#define FFT_N 64 // 256 // set to 256 point fft

#define LED_PIN 13
#define TIME_DELAY 100

#include <FFT.h> // include the library

//#include "fps_counter.hpp"

void setup() 
{ 
    pinMode(LED_PIN, OUTPUT);
    Serial.begin(9600); // use the serial port
//  TIMSK0 = 0; // turn off timer0 for lower jitter
    ADCSRA = 0xe5; // set the adc to free running mode
    ADMUX = 0x40; // use adc0
    DIDR0 = 0x01; // turn off the digital input for adc0
}

//FPSCounter counter("loop");

int16_t adjust_val(int16_t val);
#define NUM_VALUES 16
static int16_t vals[NUM_VALUES];
static uint16_t val_index = 0;

void loop() 
{ 
//  static uint16_t count = 0;
//  static uint32_t total_time = 0;
    
//  uint32_t start = micros(); // This uses timer0
    
    memset(vals, sizeof(vals), 0);

    while (1) // reduces jitter
    {
        cli();  // UDRE interrupt slows this way down on arduino1.0
        for (int i = 0; i < FFT_N * 2; i += 2) // save FFT_N samples
        {
            while (!(ADCSRA & 0x10)); // wait for adc to be ready
            ADCSRA = 0xf5; // restart adc
            uint8_t m = ADCL; // fetch adc data
            uint8_t j = ADCH; 
            int16_t k = (j << 8) | m; // form into an int
            k -= 0x0200; // form into a signed int
            k <<= 6; // form into a 16b signed int
            fft_input[i] = k; // put real data into even bins
            fft_input[i + 1] = 0; // set odd bins to 0
        }
        fft_window(); // window the data for better frequency response
        fft_reorder(); // reorder the data before doing the fft
        fft_run(); // process the data in the fft
        fft_mag_log(); // take the output of the fft
        sei(); 
        
//      total_time += micros() - start;
//
//      for (byte i = 0; i < FFT_N / 2; i++)
//      {
//          Serial.print("\t");
//          Serial.print(fft_log_out[i]); // send out the data
//      }
//      Serial.println("");
//
//      delay(500); // This uses timer0
//
//      if (count > 20)
//      {
//          Serial.println(float(total_time) / count);
//          total_time = 0;
//          count = 0;
//      }
//      count++;

        int16_t val = fft_log_out[7];

        val = adjust_val(val);

        digitalWrite(LED_PIN, HIGH);
        delay(val);
        digitalWrite(LED_PIN, LOW);
        delay(TIME_DELAY - val);

        Serial.println(val);
        delay(1000);
    }
}

int16_t adjust_val(int16_t val)
{
    vals[val_index] = val;
    val_index = (val_index+1) % NUM_VALUES;

    // Use self adjusting values
    int16_t max_val = 0xffff;
    int16_t min_val = -max_val;
    int16_t range = 0;

    for (int i=0; i<NUM_VALUES; i++) 
    {
        int16_t val = vals[i];
        if (val < min_val) 
        {
            min_val = val;
            range = max_val - min_val;
        }
        if (val > max_val) 
        {
            max_val = val;
            range = max_val - min_val;
        }
    }

    // check for a valid scale
    if (range <= 0) 
    {
        return TIME_DELAY;
    }

    val = (val - min_val) * TIME_DELAY / range;

    return val;
}
