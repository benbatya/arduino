#include <Adafruit_NeoPixel.h>

#define FFT_N 64
#define LIN_OUT 1
#include <FFT.h>

#ifndef MIC_PIN
#   define MIC_PIN 2
#endif 

#ifndef PIXEL_PIN
#   define PIXEL_PIN 6    // Digital IO pin connected to the NeoPixels.
#endif

#ifndef PIXEL_COUNT
#   define PIXEL_COUNT 60 // Num of pixels in strip
#endif

#define MIN_AMP     35

#define FASTADC     0
// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// Parameter 1 = number of pixels in strip,  neopixel stick has 8
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_RGB     Pixels are wired for RGB bitstream
//   NEO_GRB     Pixels are wired for GRB bitstream, correct for neopixel stick
//   NEO_KHZ400  400 KHz bitstream (e.g. FLORA pixels)
//   NEO_KHZ800  800 KHz bitstream (e.g. High Density LED strip), correct for neopixel stick
Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800); 

uint32_t prev_values[PIXEL_COUNT];

// Forward declare helper functions
void sample_input();
uint32_t map_amp_to_color();
uint32_t simple_map_fft_to_color();

uint8_t mode = 0;

void setup() 
{ 

#if FASTADC
    sbi(ADCSRA, ADPS2);
    cbi(ADCSRA, ADPS1);
    cbi(ADCSRA, ADPS0);
#endif

    // NOTE: that serial has to be turned off for NeoPixel and FFT to work
    Serial.begin(9600);

//  TIMSK0 = 0; // turn off timer0 for lower jitter
//  ADCSRA = 0xe5; // set the adc to free running mode
//  ADMUX = 0x40; // use adc0
//  DIDR0 = 0x01; // turn off the digital input for adc0

    strip.begin(); 
    strip.show(); // Initialize all pixels to 'off'

    memset(prev_values, sizeof(prev_values), 0);
}

void loop() 
{ 
    while (1)   // Reduces latency and keeps code sane 
    {
        uint32_t color;

//      sample_input();

        // NOTE: Look at http://www.endolith.com/wordpress/2010/09/15/a-mapping-between-musical-notes-and-colors/
        // for visualization idea

        // Also here: http://www.vamusic.info/p/home

        // TODO: make the modes switchable somehow

//      color = simple_map_fft_to_color();

        color = map_amp_to_color();
       
        // move each color down
        for (uint16_t i=0; i<PIXEL_COUNT-1; i++)
        {
            prev_values[i] = prev_values[i+1];
        }
        prev_values[PIXEL_COUNT-1] = color;
        // update the pixel colors
        for (uint16_t i=0; i<PIXEL_COUNT; i++) 
        {
            strip.setPixelColor(i, prev_values[i]);
        }
        strip.show();
    }
}

void sample_input()
{
    // Try fft_adc.pde approach
    for (int i=0; i<FFT_N*2; i+=2)
    {
        int16_t amp = analogRead(MIC_PIN);
        amp -= 0x0200; // center the amplitude
        amp <<= 6; // convert it to a int16_t
        fft_input[i] = amp;
        fft_input[i+1] = 0;
    }
}

#define INTERVAL (FFT_N/(2*3))
uint8_t sum_and_scale(const uint16_t interval_start, uint16_t* range) 
{
    uint16_t sum = 0;
    for (uint16_t i=interval_start; i<interval_start+INTERVAL; i++) 
    {   
#if LOG_OUT
        sum += fft_log_out[i];
#elif LIN_OUT
        sum += fft_lin_out[i];
#elif LIN_OUT8
        sum += fft_lin_out8[i];
#elif OCTAVE
        sum += fft_oct_out[i];
#endif
    }
    // Auto adjust the red range
    if (sum < range[0]) 
    {
        range[0] = sum;
    }
    if (sum > range[1]) 
    {
        range[1] = sum;
    }
    uint32_t temp = uint32_t(sum - range[0]) * 0xff / (range[1] - range[0]);
    return uint8_t(temp);
}

uint32_t simple_map_fft_to_color()
{
    fft_window();
    fft_reorder();
    fft_run();
#if LOG_OUT
    fft_mag_log();
#elif LIN_OUT
    fft_mag_lin();
#elif LIN_OUT8
    fft_mag_lin8();
#endif

#if OCTAVE
    fft_mag_octave();

    uint8_t red = fft_oct_out[LOG_N-3];
    uint8_t green = fft_oct_out[LOG_N-2];
    uint8_t blue = fft_oct_out[LOG_N-1];
#else
    static uint16_t red_range[] = {0xffff, 0};
    uint8_t red = sum_and_scale(0, red_range);
    static uint16_t green_range[] = {0xffff, 0};
    uint8_t green = sum_and_scale(INTERVAL, green_range);
    static uint16_t blue_range[] = {0xffff, 0};
    uint8_t blue = sum_and_scale(INTERVAL*2, blue_range);
#endif

//  {
//      Serial.print("Color = (");
//      Serial.print(r); Serial.print(", ");
//      Serial.print(g); Serial.print(", ");
//      Serial.print(b); Serial.print(", ");
//      Serial.println(")");
//   }

    return Adafruit_NeoPixel::Color(red, green, blue);
}

uint16_t amplitude()
{
    static const uint16_t sampleWindow = 50; // Sample window width in mS (50 mS = 20Hz)
    uint32_t startMillis = millis();  // Start of sample window

    uint16_t signalMax = 0;
    uint16_t signalMin = 1024;

    // collect data for 50 mS
    while (millis() - startMillis < sampleWindow)
    {
        uint16_t sample = analogRead(MIC_PIN);
        if (sample < 1024)  // toss out spurious readings
        {
            if (sample > signalMax)
            {
                signalMax = sample;  // save just the max levels
            }
            else if (sample < signalMin)
            {
                signalMin = sample;  // save just the min levels
            }
        }
    }

    uint16_t peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
    return peakToPeak;
}

// From timings, I derived that this is the min and max for the
// particular mic setup. Do a linear scale from these values to 0-255
uint32_t map_amp_to_color()
{
    // Linear amplitude display
    uint16_t amp = amplitude();
//  int16_t amp = analogRead(MIC_PIN);
//  amp -= 512; // Remove the DC offset
//  amp <<= 6;  // make it expand to fill the full signed 16 bit range

    // Used for auto scaling
    static uint16_t min_amp = 0xffff;
    static uint16_t max_amp = 0x0; //ffff;
    // Update the min and max
    if (min_amp > amp)
    {
        min_amp = amp;
    }
    if (max_amp < amp)
    {
        max_amp = amp;
    }

    Serial.print("value = "); Serial.print(amp);
    Serial.print(" ,max = "); Serial.print(max_amp);
    Serial.print(" ,min = "); Serial.println(min_amp);

    uint32_t temp = uint32_t(amp - min_amp) * 0xff / (max_amp - min_amp);
    // Run abs on temp
//  temp = (temp < 0) ? -temp: temp;

    if (temp < MIN_AMP) 
    {
        temp = 0;
    }

    // Use fixed clipping for more interesting scales
//  static const uint16_t FROM[2] = {19, 659};
//  if (amp < FROM[0])
//  {
//      amp = FROM[0];
//  } else if(amp > FROM[1])
//  {
//      amp = FROM[1];
//  }
    // Scale from MIN_RANGE-MAX_RANGE to 0-255
//  uint32_t temp = uint32_t(amp - FROM[0]) * 0xff / (FROM[1] - FROM[0]);

    uint8_t g = uint8_t(temp);

    Serial.print("final = "); Serial.println(g);

    return Adafruit_NeoPixel::Color(g, g, g);
}

// From http://www.avrfreaks.net/comment/567704#comment-567704
// Table 16*(Log2(1) thru Log2(2)), 16 values
static const uint8_t Log2Table[16]={ 0,2,3,4,5,6,7,8,9,10,11,12,13,14,14,15 }; 

// Log2(x)*16 , so log2x16(65535) = 255
uint8_t log2x16(uint16_t x){
    int8_t v;
    if(x<2) return 0;
    v=240; // 16*log2(2^15)=240
    // shift until most significant bit is set,
    // each bit-shift results in log += 16
    while ((x&0x8000)==0) { 
        x=x<<1 ; v -= 16;
    }
    // x has now form 1iii ixxx xxxx xxxx
    // get the next 4 bits =iiii and address table with it
    uint8_t i=(x>>11) & 0xf ;
    v += Log2Table[i] ;
    return v ;
}


