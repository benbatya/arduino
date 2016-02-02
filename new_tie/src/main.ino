
#define LOG_OUT 1 // use the log output function
#define FFT_N 256 // set to 256 point fft

#define LED_PIN 13
#define TIME_DELAY 100

#include <FFT.h> // include the library

#include <Adafruit_NeoPixel.h>

#ifndef MIC_PIN
#   define MIC_PIN 2
#endif 

#ifndef PIXEL_PIN
#   define PIXEL_PIN 6    // Digital IO pin connected to the NeoPixels.
#endif

#ifndef PIXEL_COUNT
#   define PIXEL_COUNT 60 // Num of pixels in strip
#endif

// Parameter 1 = number of pixels in strip,  neopixel stick has 8
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_RGB     Pixels are wired for RGB bitstream
//   NEO_GRB     Pixels are wired for GRB bitstream, correct for neopixel stick
//   NEO_KHZ400  400 KHz bitstream (e.g. FLORA pixels)
//   NEO_KHZ800  800 KHz bitstream (e.g. High Density LED strip), correct for neopixel stick
Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

void setup() 
{ 
//  pinMode(LED_PIN, OUTPUT);
//  Serial.begin(9600);
//  Serial.begin(115200); // use the serial port

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
//  ADCSRA |= _BV(ADPS2) | _BV(ADPS0);

    strip.begin();
    strip.show(); // Initialize all pixels to 'off'

    // Seed with random
    randomSeed(analogRead(0));
}

// This is low-level noise that's subtracted from each FFT output column:
static const uint8_t PROGMEM noise[128]=
{  100,90,80,80,80,80,80,80,80,80,80,70,70,70,70,70,
    70,70,70,70,70,70,70,70,70,70,80,70,70,70,70,70,
    96,106,90,70,70,70,70,70,70,70,70,70,70,70,70,70,
    70,70,70,70,70,70,70,70,70,70,80,86,80,70,70,70,
    70,105,110,85,70,70,70,70,70,70,70,70,70,70,70,70,
    70,70,70,70,70,70,70,70,70,70,70,90,90,80,70,70,
    70,70,105,105,80,70,70,70,70,70,70,70,70,70,70,70,
    70,70,70,70,70,70,70,70,70,70,70,70,95,100,70,70};

void loop() 
{   
    uint16_t prev_time = millis();

    static uint8_t color_weights[9];
    // Change the color weights
    for (int i=0; i<9; i++) 
    {
        color_weights[i] = random(256);
    }

    while (1) // reduces jitter
    {
        cli();  // UDRE interrupt slows this way down on arduino1.0
        for (int i = 0; i < FFT_N * 2; i += 2) // save FFT_N samples
        {
            static const int16_t noiseThreshold = 4;
            int16_t k = analogRead(MIC_PIN);
            k = ((k > (512-noiseThreshold)) &&
                 (k < (512+noiseThreshold))) ? 0 :
                    k - 512; // Sign-convert for FFT; -512 to +511
            k <<= 6; // form into a 16b signed int
            fft_input[i] = k; // put real data into even bins
            fft_input[i + 1] = 0; // set odd bins to 0
        }
        fft_window(); // window the data for better frequency response
        fft_reorder(); // reorder the data before doing the fft
        fft_run(); // process the data in the fft
        fft_mag_log(); // take the output of the fft
        sei();
        
        static uint8_t prev_values[FFT_N/2] = {0};
        static uint8_t output[FFT_N/2];

        for (int i=0; i<FFT_N/2; i++)
        {
            // Do a temporal filter operation
            uint8_t L = pgm_read_byte(&noise[i]);
            fft_log_out[i] = (fft_log_out[i] <= L) ? 0 : fft_log_out[i];
            int8_t diff = int8_t(fft_log_out[i]) - prev_values[i]; // get the difference
            output[i] = (diff > 70) ? 0: fft_log_out[i];  // make sure that the difference between the 
        }

        memcpy(prev_values, fft_log_out, FFT_N/2);

        static uint16_t bins[3] = { 0 };

        for (int i=0; i<3; i++)
        {
            bins[i] = 0;
        }

        uint8_t current_bin = 0;
        static const uint8_t SPLIT_POINT = FFT_N/2/3;
        for (int i=0; i<FFT_N/2; i++)
        {
            uint8_t val = output[i];
            // This is a hacky log2 calculation
            for (int j=0; j<8; j++)
            {
                if (val >= (1<<j))
                {
                    if (bins[current_bin] < 256) 
                    {
                        bins[current_bin]++; 
                    }
                }
                else
                {
                    break;
                }
            }

            // This is a hacky way of splitting up the frequency domains
            if (i%SPLIT_POINT == SPLIT_POINT-1)
            {
                current_bin++;
            }
        }

        uint16_t r = (bins[0]*color_weights[0] + bins[1]*color_weights[1] + bins[2]*color_weights[2]) / 256;
        uint16_t g = (bins[0]*color_weights[3] + bins[1]*color_weights[4] + bins[2]*color_weights[5]) / 256;
        uint16_t b = (bins[0]*color_weights[6] + bins[1]*color_weights[7] + bins[2]*color_weights[8]) / 256;

        uint32_t new_color = Adafruit_NeoPixel::Color(uint8_t(r), uint8_t(g), uint8_t(b));

        static uint32_t prev_colors[PIXEL_COUNT] = {0};
        // move each color down
        for (uint16_t i=0; i<PIXEL_COUNT-1; i++)
        {
            prev_colors[i] = prev_colors[i+1];
        }
        prev_colors[PIXEL_COUNT-1] = new_color;
        // update the pixel colors
        for (uint16_t i=0; i<PIXEL_COUNT; i++) 
        {
            strip.setPixelColor(i, prev_colors[i]);
        }
        strip.show();

        uint16_t new_time = millis();

        // Switch every 20sec
        if ((new_time - prev_time) > 5000)
        {
            // Change the color weights
            for (int i=0; i<9; i++) 
            {
                color_weights[i] = random(256);
            }

            prev_time = new_time;
        }


//      Serial.write(255); // send a start byte
//      Serial.write(bins, 3); // send out the data
//      Serial.flush();

//      delay(200);
    }
}



