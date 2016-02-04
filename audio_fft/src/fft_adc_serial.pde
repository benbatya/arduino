/*
fft_adc_serial.pde
guest openmusiclabs.com 7.7.14
example sketch for testing the fft library.
it takes in data on ADC0 (Analog0) and processes them
with the fft. the data is sent out over the serial
port at 115.2kb.
*/

#define LIN_OUT8 1 // use the log output function
#define SCALE 256
#define FFT_N 256 // set to 256 point fft

#define LED_PIN 13
#define TIME_DELAY 100

#include <FFT.h> // include the library

#if LOG_OUT
#   define FFT_FUNC fft_mag_log
#   define FFT_OUTPUT fft_log_out
#elif LIN_OUT8
#   define FFT_FUNC fft_mag_lin8
#   define FFT_OUTPUT fft_lin_out8
#else
#error "This is invalid!!"
#endif

#define DISPLAY_OUTPUT 1

#ifndef DISPLAY_OUTPUT
uint32_t prev_time;
static uint8_t max_vals[FFT_N/2];
#endif

void setup() 
{ 
//  pinMode(LED_PIN, OUTPUT);
//  Serial.begin(9600);
    Serial.begin(115200); // use the serial port

    analogReference(DEFAULT);
//  TIMSK0 = 0; // turn off timer0 for lower jitter
//  ADCSRA = 0xe5; // set adc on, set the adc to free running mode, prescaler=32

//    ADMUX  = _BV(REFS0) | (MIC_PIN - 8);
//    ADCSRB = _BV(MUX5);
//    DIDR2 = 1<<(MIC_PIN-8); // turn off the digital input for adc2
//    ADCSRA = _BV(ADEN)  | // ADC enable
//            _BV(ADSC)  | // ADC start
//            _BV(ADATE) | // Auto trigger
////          _BV(ADIE)  | // Interrupt enable
//            _BV(ADPS1) | _BV(ADPS0); // 64:1 / 13 = 9615 Hz
//  ADCSRA |= _BV(ADPS0); // | _BV(ADPS0);
    // Change the prescaler to 8 to get a better sampling rate
    ADCSRA &= ~0x07;
    ADCSRA |= _BV(ADPS1) | _BV(ADPS0);

#ifndef DISPLAY_OUTPUT

    prev_time = millis();

//  uint32_t total = 0;
//  uint32_t count = 0;

    for (int i=0; i<FFT_N/2; i++) 
    {
        max_vals[i] = 0;
    }
#endif

}

//FPSCounter counter("loop");

//int16_t adjust_val(int16_t val);
//#define NUM_VALUES 16
//static int16_t vals[NUM_VALUES];
//static uint16_t val_index = 0;

//uint8_t log2x16(uint16_t x);
//

#if DISPLAY_OUTPUT

//static const uint8_t PROGMEM noise[128] = {
//    117, 108, 97, 102, 99, 102, 103, 105, 110, 103, 96, 96, 91, 93, 93, 107
//    , 111, 101, 87, 87, 85, 79, 87, 104, 105, 85, 80, 79, 68, 68, 79, 96
//    , 94, 71, 71, 62, 62, 62, 76, 90, 85, 63, 58, 66, 61, 58, 70, 84
//    , 75, 55, 58, 58, 55, 52, 69, 76, 68, 51, 55, 51, 53, 53, 67, 70
//    , 57, 48, 50, 52, 47, 47, 60, 64, 54, 50, 51, 47, 45, 48, 56, 59
//    , 51, 48, 45, 50, 44, 44, 53, 55, 47, 45, 46, 47, 46, 47, 50, 51
//    , 46, 42, 45, 43, 43, 45, 47, 46, 41, 44, 42, 45, 44, 45, 48, 43
//    , 43, 42, 43, 45, 43, 44, 46, 45, 41, 41, 45, 45, 41, 43, 43, 43
//};

static const uint8_t PROGMEM noise[128] = {
    20, 12, 6, 9, 10, 8, 7, 7, 10, 7, 5, 4, 5, 6, 5, 8
    , 11, 5, 3, 3, 4, 3, 3, 7, 8, 4, 2, 2, 1, 1, 2, 6
    , 5, 1, 1, 1, 1, 1, 1, 4, 4, 0, 0, 0, 0, 0, 1, 3
    , 2, 0, 0, 0, 0, 0, 1, 2, 1, 0, 0, 0, 0, 0, 0, 1
    , 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    , 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    , 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    , 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};


#else

static const uint8_t PROGMEM  noise[128] = { 
    // This is low-level noise that's subtracted from each FFT output column:
    00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,
    00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,
    00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,
    00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,
    00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,
    00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,
    00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,
    00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00
};
#endif

//uint16_t amplitude();

void loop() 
{  
//  while (1) // reduces jitter
    {
//      uint32_t start_time = micros();

        for (int i = 0; i < FFT_N; i++) // save FFT_N samples
        {
            int16_t k = analogRead(MIC_PIN);

            static const int16_t noiseThreshold = 4;
            k = ((k > (512-noiseThreshold)) &&
                 (k < (512+noiseThreshold))) ? 0 :
                    k - 512; // Sign-convert for FFT; -512 to +511
            k <<= 6; // form into a 16b signed int
            fft_input[i*2] = k; // put real data into even bins
            fft_input[i*2 + 1] = 0; // set odd bins to 0
        }

//      total += micros() - start_time;
//      count++;

        fft_window(); // window the data for better frequency response
        fft_reorder(); // reorder the data before doing the fft
        fft_run(); // process the data in the fft
        FFT_FUNC(); // take the output of the fft
        
//      static uint8_t prev_values[FFT_N/2] = {0};

#if DISPLAY_OUTPUT

        for (int i=0; i<FFT_N/2; i++)
        {
            // Do a temporal filter operation
            uint8_t L = pgm_read_byte(&noise[i]);
            FFT_OUTPUT[i] = (FFT_OUTPUT[i] <= L) ? 0 : FFT_OUTPUT[i];
//          int8_t diff = int8_t(fft_log_out[i]) - prev_values[i]; // get the difference
//          output[i] = (diff > 70) ? 0: fft_log_out[i];  // make sure that the difference between the
        }
//
//      memcpy(prev_values, fft_log_out, FFT_N/2);

        Serial.write(255); // send a start byte
        Serial.write(FFT_OUTPUT, FFT_N/2); // send out the data
//      Serial.write(output, FFT_N/2); // send out the data

#else

//      if (count > 50)
//      {
//          float avg = float(total) / count;
//          total = 0;
//          count = 0;
//          Serial.print("average time = "); Serial.print(avg);
//          float hz = avg / FFT_N;
//          hz = 1000000.0f/hz;
//          Serial.print(", Hz = "); Serial.println(hz);
//      }

        for (int i=0; i<FFT_N/2; i++)
        {
            if (max_vals[i] < FFT_OUTPUT[i])
            {
                max_vals[i] = FFT_OUTPUT[i];
            }
        }

        if (millis() - prev_time > 5000)
        {
            Serial.print("static const uint8_t PROGMEM noise["); Serial.print(FFT_N/2); Serial.print("] = {");
            const char* sep = "";
            for (int i=0; i<FFT_N/2; i++)
            {
                if (i%16 == 0)
                {
                    Serial.println(""); Serial.print("    ");
                }
                Serial.print(sep); Serial.print(max_vals[i]);
                sep = ", ";
            }
            Serial.println(""); Serial.println("};");
            prev_time = millis();
        }
#endif

//      Serial.flush();
    }
}

//uint16_t amplitude()
//{
//    static const uint16_t sampleWindow = 50; // Sample window width in mS (50 mS = 20Hz)
//    uint32_t startMillis = micros();  // Start of sample window
//
//    uint16_t signalMax = 0;
//    uint16_t signalMin = 1024;
//
//    // collect data for 50 mS
//    while (micros() - startMillis < sampleWindow)
//    {
//        uint16_t sample = analogRead(MIC_PIN);
//        if (sample < 1024)  // toss out spurious readings
//        {
//            if (sample > signalMax)
//            {
//                signalMax = sample;  // save just the max levels
//            }
//            else if (sample < signalMin)
//            {
//                signalMin = sample;  // save just the min levels
//            }
//        }
//    }
//
//    uint16_t peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
//    return peakToPeak;
//}

