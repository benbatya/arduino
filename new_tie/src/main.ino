/**
 * This is code for the tie
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

#include <Adafruit_NeoPixel.h>

#include <EEPROM.h>

#ifndef MIC_PIN
#   define MIC_PIN 2
#endif 

#ifndef PIXEL_PIN
#   define PIXEL_PIN 6    // Digital IO pin connected to the NeoPixels.
#endif

#ifndef PIXEL_COUNT
#   define PIXEL_COUNT 60 // Num of pixels in strip
#endif

#define SWITCH_TIME 5000

#define GEA(val, exp, el) ((val >= 1<<exp) ? exp : el) 

// Parameter 1 = number of pixels in strip,  neopixel stick has 8
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_RGB     Pixels are wired for RGB bitstream
//   NEO_GRB     Pixels are wired for GRB bitstream, correct for neopixel stick
//   NEO_KHZ400  400 KHz bitstream (e.g. FLORA pixels)
//   NEO_KHZ800  800 KHz bitstream (e.g. High Density LED strip), correct for neopixel stick
Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

// The running modes
enum class MODE : byte
{
    AMP = 0,
    FFT = 1,
    VU = 2,
    EQ = 3,
    LEN,

    MIN = AMP,
    MAX = LEN-1
};

static MODE current_mode = MODE::FFT;

//#define DEBUG 1

static int32_t prev_time;
static uint8_t color_weights[9];

void switch_colors();

void setup() 
{ 
    // Change the prescaler to 16 to get a better sampling rate
    ADCSRA &= ~0x07;
    ADCSRA |= _BV(ADPS2); // prescaler = 16

    analogReference(DEFAULT);
        
    strip.begin();
    strip.show(); // Initialize all pixels to 'off'

    // Seed with random
    randomSeed(analogRead(0));

#ifdef DEBUG 
    Serial.begin(115200); // use the serial port
#else
    if (EEPROM.length() > 0)  // Only try to use EEPROM if there is some on the board
    {
        static const int EEPROM_address = 0;

        // Get the previous mode. If it's valid, set current_mode the next mode % MODE::LEN
        MODE value = MODE(EEPROM.read(EEPROM_address));
        if (MODE::MIN <= value && value <= MODE::MAX) 
        {
            current_mode = MODE((uint8_t(value)+1) % uint8_t(MODE::LEN));
        }

        EEPROM.write(EEPROM_address, byte(current_mode));
    }

    // Show a red pixel for 2 seconds if we are displaying AMP mode
    switch (current_mode) 
    {
    case MODE::AMP:
        strip.setPixelColor(0, 255, 0, 0);
        break;
    case MODE::FFT:
        strip.setPixelColor(0, 0, 255, 0);
        break;
    case MODE::VU:
        strip.setPixelColor(0, 0, 0, 255);
        break;
    case MODE::EQ:
        strip.setPixelColor(0, 0, 127, 127);
        break;
    default:
        break;
    }

    // Drop the brightness by half
    strip.setBrightness(127);
    strip.show();
    delay(2000);

    prev_time = millis();

    // Init the color weights
    switch_colors();
#endif
}

void flow_modes();
void vu_mode();
void eq_mode();

void loop() 
{   
#ifdef DEBUG
    eq_mode();
#else
    switch (current_mode) 
    {
    case MODE::FFT:
    case MODE::AMP:
        flow_modes(); 
        break;
    case MODE::VU:
        vu_mode();
        break;
    case MODE::EQ:
        eq_mode();
        break;
    default:
        break;
    }

    // Halve the brightness
    strip.setBrightness(127);
    strip.show();  

    uint16_t new_time = millis();

    // Switch every SWITCH_TIME
    if ((new_time - prev_time) > SWITCH_TIME)
    {
        switch_colors();

        prev_time = new_time;
    }
#endif
}


uint32_t fft_to_color();
uint32_t amp_to_color();

void flow_modes()
{
    uint32_t color;

    if (current_mode == MODE::FFT) 
    {
        color = fft_to_color();
    } else
    {
        color = amp_to_color();
    }

    static uint32_t prev_colors[PIXEL_COUNT] = {0};
    // move each color down
    for (uint16_t i=0; i<PIXEL_COUNT-1; i++)
    {
        prev_colors[i] = prev_colors[i+1];
    }
    prev_colors[PIXEL_COUNT-1] = color;
    // update the pixel colors
    for (uint16_t i=0; i<PIXEL_COUNT; i++) 
    {
        strip.setPixelColor(i, prev_colors[i]);
    }  

//      Serial.write(255); // send a start byte
//      Serial.write(bins, 3); // send out the data
//      Serial.flush();

//      delay(200);
}

// Shared static functions and data

#define INPUT_FLOOR 19 //Lower range of analogRead input
#define INPUT_CEILING 600 //Max range of analogRead input, the lower the value the more sensitive (1023 = max)

float fscale(float originalMin, float originalMax, float newBegin, float
             newEnd, float inputValue, float curve);
uint16_t amplitude();

// start Mode::FFT functionality

// This is low-level noise that's subtracted from each FFT output column:
static const uint8_t PROGMEM noise[128] = {
    15, 10, 8, 4, 5, 7, 6, 7, 5, 5, 6, 9, 8, 5, 4, 4
    , 5, 4, 4, 3, 4, 4, 7, 11, 6, 2, 2, 2, 2, 3, 2, 2
    , 2, 4, 8, 7, 3, 2, 1, 1, 1, 1, 1, 1, 2, 5, 6, 3
    , 1, 0, 0, 0, 0, 0, 0, 0, 2, 4, 3, 0, 0, 0, 0, 0
    , 0, 0, 0, 0, 3, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1
    , 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0
    , 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    , 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

// Returns an array of uint8_t of size FFT_N/2
uint8_t* calc_fft_input()
{
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
    FFT_FUNC(); // take the output of the fft
    
    for (int i=0; i<FFT_N/2; i++)
    {
        // Do a temporal filter operation
        uint8_t L = pgm_read_byte(&noise[i]);
        FFT_OUTPUT[i] = (FFT_OUTPUT[i] <= L) ? 0 : FFT_OUTPUT[i];
    }

    return FFT_OUTPUT;
}

uint32_t fft_to_color()
{
    uint8_t* output = calc_fft_input();

    static uint16_t bins[3] = { 0 };

    for (int i=0; i<3; i++)
    {
        bins[i] = 0;
    }

    uint8_t current_bin = 0;
    // Split the bands into roughly low, mid, and high
    static const uint8_t BAND_END[3] = {19, 59, 127};
    for (int i=0; i<FFT_N/2; i++)
    {
        uint8_t val = output[i];

        bins[current_bin] += GEA(val, 7, GEA(val, 6, GEA(val, 5, GEA(val, 4, GEA(val, 3, GEA(val, 2, GEA(val, 1, 0)))))));

        // This is a hacky way of splitting up the frequency domains
        if (i >= BAND_END[current_bin])
        {
            current_bin++;
        }
    }

    uint16_t r = (bins[0]*color_weights[0] + bins[1]*color_weights[1] + bins[2]*color_weights[2]) >> 8;
    uint16_t g = (bins[0]*color_weights[3] + bins[1]*color_weights[4] + bins[2]*color_weights[5]) >> 8;
    uint16_t b = (bins[0]*color_weights[6] + bins[1]*color_weights[7] + bins[2]*color_weights[8]) >> 8;

    uint32_t new_color = Adafruit_NeoPixel::Color(uint8_t(r), uint8_t(g), uint8_t(b));
    return new_color;
}

// start Mode::AMP functionality

// From timings, I derived that this is the min and max for the
// particular mic setup. Do a linear scale from these values to 0-255
uint32_t amp_to_color()
{
    // Linear amplitude display
    uint16_t amp = amplitude();


    //Scale the input logarithmically instead of linearly
    uint16_t w = fscale(INPUT_FLOOR, INPUT_CEILING, 0, 255, amp, 2); 

    uint16_t r = (w*color_weights[0]) >> 8;
    uint16_t g = (w*color_weights[1]) >> 8;
    uint16_t b = (w*color_weights[2]) >> 8;

//  Serial.print("final = "); Serial.println(g);

    return Adafruit_NeoPixel::Color(uint8_t(r), uint8_t(g), uint8_t(b));
//  return Adafruit_NeoPixel::Color(uint8_t(w), uint8_t(w), uint8_t(w));
}

// start MODE::VU code
// NOTE: taken from Adafruit's original tie code
#define SAMPLE_WINDOW   10  // Sample window for average level
#define PEAK_HANG 24 //Time of pause before peak dot falls
#define PEAK_FALL 4 //Rate of falling peak dot

byte peak = 16;      // Peak level of column; used for falling dots
//unsigned int sample;

byte dotCount = 0;  //Frame counter for peak dot
byte dotHangCount = 0; //Frame counter for holding peak dot

void drawLine(uint8_t from, uint8_t to, uint32_t c);
uint32_t Wheel(byte WheelPos);

void vu_mode()
{
    float peakToPeak = amplitude();

    //Fill the strip with rainbow gradient
    for (uint16_t i = 0; i <= strip.numPixels() - 1; i++) 
    {
        strip.setPixelColor(i, Wheel(map(i, 0, strip.numPixels() - 1, 30, 150)));
    }
    
    
    //Scale the input logarithmically instead of linearly
    uint16_t c = fscale(INPUT_FLOOR, INPUT_CEILING, strip.numPixels(), 0, peakToPeak, 2); 
    
    if (c < peak) 
    {
        peak = c;        // Keep dot on top
        dotHangCount = 0;    // make the dot hang before falling
    }
    if (c <= strip.numPixels()) // Fill partial column with off pixels
    {
        drawLine(strip.numPixels(), strip.numPixels() - c, strip.Color(0, 0, 0));
    }
    
    // Set the peak dot to match the rainbow gradient
    uint16_t y = strip.numPixels() - peak; 
    
    strip.setPixelColor(y - 1, Wheel(map(y, 0, strip.numPixels() - 1, 30, 150))); 
    
    // Frame based peak dot animation
    if (dotHangCount > PEAK_HANG) //Peak pause length
    {
        if (++dotCount >= PEAK_FALL) //Fall rate 
        {
            peak++; 
            dotCount = 0;
        }
    } 
    else 
    {
        dotHangCount++; 
    }
}

void eq_mode()
{
    uint8_t* output = calc_fft_input();

    static const uint8_t FREQ_PER_PIXEL = FFT_N/(2*PIXEL_COUNT);

    static uint16_t bins[PIXEL_COUNT] = { 0 };

    for (int i=0; i<PIXEL_COUNT; i++)
    {
        bins[i] = 0;
    }

    uint8_t current_bin = 0;
    for (uint16_t i=0; i<FFT_N/2; i++)
    {
        uint8_t val = output[i];
        
        bins[current_bin] += GEA(val, 7, GEA(val, 6, GEA(val, 5, GEA(val, 4, GEA(val, 3, GEA(val, 2, GEA(val, 1, 0)))))));

        // This is a hacky way of splitting up the frequency domains
        if ((i+1)%FREQ_PER_PIXEL == 0)
        {
            current_bin++;
        }
    }

    // Split the bands into roughly low, mid, and high
    static const uint8_t BAND_END[3] = {2, 8, 16};

    uint8_t band = 0;
    for (uint8_t i=0; i<PIXEL_COUNT; i++) 
    {
        uint16_t val = bins[i];

        uint16_t r = (val*color_weights[band*3+0]) >> 8;
        uint16_t g = (val*color_weights[band*3+1]) >> 8;
        uint16_t b = (val*color_weights[band*3+2]) >> 8;
        strip.setPixelColor(i, uint8_t(r), uint8_t(g), uint8_t(b));

        if (i >= BAND_END[band])
        {
            band++;
        }
    }

#ifdef DEBUG
    Serial.write(255);

//  static uint8_t data[PIXEL_COUNT];
//  for (uint8_t i=0; i<PIXEL_COUNT; i++)
//  {
//      uint8_t val = bins[i];
//      val = val==255? 254: val;
//      data[i] = val;
//  }
    Serial.write(output, FFT_N/2);
#endif

}

//Used to draw a line between two points of a given color
void drawLine(uint8_t from, uint8_t to, uint32_t c) 
{ 
    uint8_t fromTemp; 
    if (from > to) 
    {
        fromTemp = from; 
        from = to; 
        to = fromTemp;
    }
    for (int i = from; i <= to; i++) 
    {
        strip.setPixelColor(i, c);
    }
}


float fscale(float originalMin, float originalMax, float newBegin, float
             newEnd, float inputValue, float curve) 
{ 
    
    float OriginalRange = 0; 
    float NewRange = 0; 
    float zeroRefCurVal = 0; 
    float normalizedCurVal = 0; 
    float rangedValue = 0; 
    boolean invFlag = 0; 
    
    
    // condition curve parameter
    // limit range
    
    if (curve > 10) curve = 10; 
    if (curve < -10) curve = -10; 
    
    curve = (curve * -.1); // - invert and scale - this seems more intuitive - postive numbers give more weight to high end on output 
    curve = pow(10, curve); // convert linear scale into lograthimic exponent for other pow function
    
    /*
     Serial.println(curve * 100, DEC);   // multply by 100 to preserve resolution  
     Serial.println(); 
     */
    
    // Check for out of range inputValues
    if (inputValue < originalMin) 
    {
        inputValue = originalMin;
    }
    if (inputValue > originalMax) 
    {
        inputValue = originalMax;
    }
    
    // Zero Refference the values
    OriginalRange = originalMax - originalMin; 
    
    if (newEnd > newBegin) 
    {
        NewRange = newEnd - newBegin;
    }
    else 
    {
        NewRange = newBegin - newEnd; 
        invFlag = 1;
    }
    
    zeroRefCurVal = inputValue - originalMin; 
    normalizedCurVal  =  zeroRefCurVal / OriginalRange;   // normalize to 0 - 1 float
    
    // Check for originalMin > originalMax  - the math for all other cases i.e. negative numbers seems to work out fine 
    if (originalMin > originalMax) 
    {
        return 0;
    }
    
    if (invFlag == 0) 
    {
        rangedValue =  (pow(normalizedCurVal, curve) * NewRange) + newBegin;
        
    }
    else     // invert the ranges
    {
        rangedValue =  newBegin - (pow(normalizedCurVal, curve) * NewRange); 
    }
    
    return rangedValue;
}


// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) 
{ 
    if (WheelPos < 85) 
    {
        return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
    } 
    else if (WheelPos < 170) 
    {
        WheelPos -= 85; 
        return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
    } 
    else 
    {
        WheelPos -= 170; 
        return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
    }
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

void switch_colors()
{
    uint16_t color_idx0 = random(256);
    uint16_t color_idx1 = (color_idx0 + 85) % 256;
    uint16_t color_idx2 = (color_idx0 + 171) % 256;

    uint32_t color0 = Wheel(byte(color_idx0));
    uint32_t color1 = Wheel(byte(color_idx1));
    uint32_t color2 = Wheel(byte(color_idx2));

    color_weights[0] = (color0 >> 16) & 0xff;
    color_weights[1] = (color0 >> 8) & 0xff;
    color_weights[2] = (color0 >> 0) & 0xff;
    color_weights[3] = (color1 >> 16) & 0xff;
    color_weights[4] = (color1 >> 8) & 0xff;
    color_weights[5] = (color1 >> 0) & 0xff;
    color_weights[6] = (color2 >> 16) & 0xff;
    color_weights[7] = (color2 >> 8) & 0xff;
    color_weights[8] = (color2 >> 0) & 0xff;
}
