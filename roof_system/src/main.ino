/**
 * This is code for the tie
 */

#define LIN_OUT8 1 // use the log output function
#define SCALE 256
#define FFT_N 128 // set to 256 point fft

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
#define EEPROM_mode_address 0
#define EEPROM_brightness_address 1

#include "BluefruitConfig.h"

#include "util_funcs.hpp"

#ifndef MIC_PIN
#   define MIC_PIN 2
#endif 

#ifndef PIXEL_PIN
#   define PIXEL_PIN 6    // Digital IO pin connected to the NeoPixels.
#endif

#ifndef PIXEL_COUNT
#   define PIXEL_COUNT 60 // Num of pixels in strip
#endif
//
//
//static const uint8_t PROGMEM noise[64] = {
//    0, 0, 0, 0, 1, 2, 2, 1, 2, 2, 2, 1, 2, 1, 2, 1, 0, 0, 2, 2, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1
//    , 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 2, 2, 1, 0, 0, 0, 0, 0, 1, 2, 2, 1, 0, 0, 0, 0, 0, 0, 1
//};

uint32_t SWITCH_TIME = 500;

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

// The current display mode 
static MODE current_mode = MODE::FFT;

// A standby mode if there's no voice input detected for 30 seconds
static bool in_standby_mode = false;
static uint32_t standby_mode_color;
static uint32_t prev_standby_time;
#define STANDBY_MODE_TIME 30000
#define STANDBY_TRIGGER_AMOUNT (FFT_N*120)  // Multiplier found empirically

static uint32_t prev_time;
static uint16_t color_idx0;
static uint8_t color_weights[9];
#define BRIGHTNESS_INC      16
static uint8_t brightness = 127;

void switch_colors(int16_t inc = 1);
void eq_setup_mode();

void setup() 
{ 
    CONFIG();

    // Change the prescaler to get a better sampling rate
    ADCSRA &= ~0x07;
    ADCSRA |= _BV(ADPS2) | _BV(ADPS0); // prescaler = 32

    analogReference(DEFAULT);
        
    strip.begin();
    strip.show(); // Initialize all pixels to 'off'

    // Seed with random
    randomSeed(analogRead(0));

#ifdef DEBUG 
    current_mode = MODE::EQ;
#else
    if (EEPROM.length() > 0)  // Only try to use EEPROM if there is some on the board
    {
        // Get the previous mode. If it's valid, set current_mode the next mode % MODE::LEN
        MODE value = MODE(EEPROM.read(EEPROM_mode_address));
        if (MODE::MIN <= value && value <= MODE::MAX) 
        {
            current_mode = MODE((uint8_t(value)+1) % uint8_t(MODE::LEN));
        }

        EEPROM.update(EEPROM_mode_address, byte(current_mode));

        brightness = EEPROM.read(EEPROM_brightness_address);
    }

#endif

    // Setup the ble
    ble_setup();

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
        eq_setup_mode();
        strip.setPixelColor(0, 0, 127, 127);
        break;
    default:
        break;
    }

    // Drop the brightness by half
    strip.setBrightness(brightness);
    strip.show();
    delay(2000);

    prev_time = millis();
    prev_standby_time = prev_time;

    // Start the color at a random color
    color_idx0 = random(256);
    // Init the color weights
    switch_colors();
}

void flow_modes();
void vu_mode();
void eq_mode();
void standby_mode();

int16_t* sample();

void loop() 
{   

//  uint32_t total_amp = 0;
//  for (int i=0; i<FFT_N; i++)
//  {
//      total_amp += abs(fft_input[i*2]);
//  }
//  if (total_amp >= STANDBY_TRIGGER_AMOUNT)
//  {
//      prev_standby_time = new_time;
//      in_standby_mode = false;
//  } else if(!in_standby_mode && (new_time - prev_standby_time > STANDBY_MODE_TIME))
//  {
//      in_standby_mode = true;
//  }

    if (in_standby_mode) 
    {
        standby_mode();
    } else
    {
        sample();

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
    }

    strip.setBrightness(brightness);

    // Show the strip
    strip.show();     

    uint16_t new_time = millis();

    // Switch every SWITCH_TIME if not in standby mode
    if ((new_time - prev_time) > SWITCH_TIME && !in_standby_mode)
    {
        switch_colors();

        prev_time = new_time;

//      PRINT(F("total_amp = "));
//      PRINT(total_amp);
    }

    if( ble_update() )
    {
        uint32_t color;
        if (ble_get_color(color)) 
        {
            in_standby_mode = true;
            standby_mode_color = color;
        } 

        uint8_t button;
        bool pressed;
        if(ble_get_button(button, pressed))
        {
            // React on the release only
            if (!pressed) 
            {
                if (1 <= button && button <= 4) 
                {
                    in_standby_mode = false;

                    current_mode = MODE(byte(button-1) % byte(MODE::LEN));
                    // update the current_mode in the EEPROM
                    EEPROM.update(EEPROM_mode_address, byte(current_mode));

                    switch (current_mode) 
                    {
                    case(MODE::EQ):
                        eq_setup_mode();
                        break;
                    default:
                        break;
                    }
                } else if(button == 5) // up
                {
                    if (brightness < (255-BRIGHTNESS_INC)) 
                    {
                        brightness += BRIGHTNESS_INC; 
                    }
                    EEPROM.update(EEPROM_brightness_address, byte(brightness));
                } else if(button == 6) // down
                {
                    if (brightness > BRIGHTNESS_INC) 
                    {
                        brightness -= BRIGHTNESS_INC; 
                    }
                    EEPROM.update(EEPROM_brightness_address, byte(brightness));
                } else if(button == 7) // left
                {
                    switch_colors(-16);
                } else if(button == 8) // right
                {
                    switch_colors(16);
                    prev_time = new_time;
                }
            }
        }
    }
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
    for (uint16_t i=PIXEL_COUNT-1; i>0; i--)
    {
        prev_colors[i] = prev_colors[i-1];
    }
    prev_colors[0] = color;
    // update the pixel colors
    for (uint16_t i=0; i<PIXEL_COUNT; i++) 
    {
        strip.setPixelColor(i, prev_colors[i]);
    }  

#if 0 // DEBUG
//      Serial.write(255); // send a start byte
//      Serial.write(bins, 3); // send out the data
//      Serial.flush();

//      delay(200);
#endif
}

// Shared static functions and data
//
#define INPUT_FLOOR 19 //Lower range of analogRead input
#define INPUT_CEILING 600 //Max range of analogRead input, the lower the value the more sensitive (1023 = max)

float fscale(float originalMin, float originalMax, float newBegin, float
             newEnd, float inputValue, float curve);
//
//// start Mode::FFT functionality
//

int16_t* sample()
{
    for (int i = 0; i < FFT_N * 2; i += 2) // save FFT_N samples
    {
        static const int16_t noiseThreshold = 6;
        int16_t k = analogRead(MIC_PIN);
        k = ((k > (512-noiseThreshold)) &&
             (k < (512+noiseThreshold))) ? 0 :
                k - 512; // Sign-convert for FFT; -512 to +511
        k <<= 6; // form into a 16b signed int
        fft_input[i] = k; // put real data into even bins
        fft_input[i + 1] = 0; // set odd bins to 0
    }

    return fft_input;
}

// Returns an array of uint8_t of size FFT_N/2
uint8_t* calc_fft_input()
{
    fft_window(); // window the data for better frequency response
    fft_reorder(); // reorder the data before doing the fft
    fft_run(); // process the data in the fft
    FFT_FUNC(); // take the output of the fft
    
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

    // For the Frequency modes (FFT and EQ), use this as a rough division between low, mid and high bands
    static const uint8_t BAND_END[3] = {9, 27, 63};

    uint8_t current_bin = 0;
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

    uint16_t r = (bins[0]*color_weights[0] + bins[1]*color_weights[1] + bins[2]*color_weights[2]) >> (8-2);
    uint16_t g = (bins[0]*color_weights[3] + bins[1]*color_weights[4] + bins[2]*color_weights[5]) >> (8-2);
    uint16_t b = (bins[0]*color_weights[6] + bins[1]*color_weights[7] + bins[2]*color_weights[8]) >> (8-2);

    uint32_t new_color = Adafruit_NeoPixel::Color(uint8_t(r), uint8_t(g), uint8_t(b));
    return new_color;
}

// start Mode::AMP functionality

// From timings, I derived that this is the min and max for the
// particular mic setup. Do a linear scale from these values to 0-255
uint32_t amp_to_color()
{
    // Linear amplitude display
    int16_t max_val, min_val;
    max_val = min_val = fft_input[0];
    for (int i=1; i<FFT_N; i++)
    {
        int16_t val = fft_input[i*2];
        if (val < min_val) { min_val = val; }
        else if (val > max_val) { max_val = val; }
    }
    uint16_t w = max_val - min_val;

    //Scale the input logarithmically instead of linearly
    w = fscale(1200, 32000, 0, 255, w, 2);

#if 0 // DEBUG
    uint16_t new_time = millis();

    // Switch every SWITCH_TIME
    if ((new_time - prev_time) > (SWITCH_TIME-10))
    {
        Serial.print("AMP mode: value = "); Serial.println(w);
    }
#endif

    uint16_t r;
    uint16_t g;
    uint16_t b;

    if (w > 150) 
    {
        r = (w*color_weights[0]) >> 8;
        g = (w*color_weights[1]) >> 8;
        b = (w*color_weights[2]) >> 8;
    } else if( w > 45)
    {
        r = (w*color_weights[3]) >> 8;
        g = (w*color_weights[4]) >> 8;
        b = (w*color_weights[5]) >> 8;
    } else
    {
        r = (w*color_weights[6]) >> 8;
        g = (w*color_weights[7]) >> 8;
        b = (w*color_weights[8]) >> 8;
    }

    return Adafruit_NeoPixel::Color(uint8_t(r), uint8_t(g), uint8_t(b));
}

// start MODE::VU code
// NOTE: taken from Adafruit's original tie code
#define SAMPLE_WINDOW   10  // Sample window for average level
#define PEAK_HANG 24 //Time of pause before peak dot falls
#define PEAK_FALL 4 //Rate of falling peak dot

void drawLine(uint8_t from, uint8_t to, uint32_t c);
uint32_t Wheel(byte WheelPos);

void vu_mode()
{
    int16_t max_val, min_val;
    max_val = min_val = fft_input[0];
    for (int i=1; i<FFT_N; i++)
    {
        int16_t val = fft_input[i*2];
        if (val < min_val) { min_val = val; }
        else if (val > max_val) { max_val = val; }
    }
    uint16_t w = max_val - min_val;
    w >>= (6-2);
    
    //Scale the input logarithmically instead of linearly
    uint16_t c = fscale(INPUT_FLOOR, INPUT_CEILING, 0, strip.numPixels(), w, 2); 

    static byte peak = 16;      // Peak level of column; used for falling dots
    static byte dotCount = 0;  //Frame counter for peak dot
    static byte dotHangCount = 0; //Frame counter for holding peak dot

    if (c >= peak) 
    {
        peak = c;        // Keep dot on top
        dotHangCount = 0;    // make the dot hang before falling
    }
    
    const uint8_t bottom_color = 30;
    const uint8_t top_color = 150;

    //Fill the strip with rainbow gradient
    for (uint16_t i = 0; i < c; i++) 
    {

        strip.setPixelColor(i, Wheel((map(i, 0, strip.numPixels() - 1, bottom_color, top_color) + color_idx0) % 256));
    }
    // Fill the rest with black
    drawLine(c, strip.numPixels(), strip.Color(0, 0, 0));
    
    // Set the peak dot to match the rainbow gradient
    strip.setPixelColor(peak, Wheel((map(peak, 0, strip.numPixels() - 1, bottom_color, top_color) + color_idx0) % 256)); 
    
    // Frame based peak dot animation
    if (dotHangCount > PEAK_HANG) //Peak pause length
    {
        if (++dotCount >= PEAK_FALL) //Fall rate 
        {
            peak--; 
            dotCount = 0;
        }
    } 
    else 
    {
        dotHangCount++; 
    }
}

static uint16_t eq_pixels[PIXEL_COUNT];

void eq_setup_mode()
{
    // Clear the pixel values
    for (int i=0; i<PIXEL_COUNT; i++)
    {
        eq_pixels[i] = 0;
    }
}


void eq_mode()
{
    uint8_t* output = calc_fft_input();

    // This maps the bands to the pixels
#if PIXEL_COUNT == 60
    static const uint8_t FREQ_PER_PIXEL[3] = {1, 1, 1};
    static const uint8_t BAND_PIXEL_END[3] = {19, 51, 59};
#else
    // THIS is NOT tuned YET!
//  static const uint8_t FREQ_PER_PIXEL[3] = {8, 8, 8};
//  static const uint8_t BAND_PIXEL_END[3] = {3, 13, 15};
#endif

    uint8_t band = 0;
    uint8_t freq = 0;
    for (uint8_t current_pixel=0; current_pixel<PIXEL_COUNT; current_pixel++) 
    {
        eq_pixels[current_pixel] = (uint32_t(eq_pixels[current_pixel]) * 15) / 16;

        for (int j=0; j<FREQ_PER_PIXEL[band]; j++) 
        {
            uint8_t val = output[freq];
            eq_pixels[current_pixel] += GEA(val, 7, GEA(val, 6, GEA(val, 5, GEA(val, 4, GEA(val, 3, GEA(val, 2, GEA(val, 1, 0)))))));
            freq++;
        }

        if (current_pixel >= BAND_PIXEL_END[band]) 
        {
            band++;
        }
    }

    band = 0;
    for (uint8_t i=0; i<PIXEL_COUNT; i++) 
    {
        uint16_t val = eq_pixels[i];

        uint16_t r = (val*color_weights[band*3+0]) >> (8-5);
        uint16_t g = (val*color_weights[band*3+1]) >> (8-5);
        uint16_t b = (val*color_weights[band*3+2]) >> (8-5);
        strip.setPixelColor(i, uint8_t(r), uint8_t(g), uint8_t(b));

        if (i >= BAND_PIXEL_END[band])
        {
            band++;
        }
    }

#if 0 // DEBUG
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

void standby_mode()
{
//  static const uint16_t wait = 50;
//  static uint8_t q = 0;
//  static uint8_t color = 0;
//
//  delay(wait);
//
//  drawLine(0, PIXEL_COUNT, 0);
//
//  //Theatre-style crawling lights with rainbow effect
//  for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
//      uint32_t c = Wheel( (i+color) % 255 );
//      strip.setPixelColor(i+q, c);    //turn every third pixel on
//      strip.setPixelColor(i+q+1, c);    //turn every third pixel on
//  }
//
//  q = (q+1) % 3;
//  color = (color+1);

    for (uint16_t i=0; i < strip.numPixels(); i++) {
        strip.setPixelColor(i, standby_mode_color);
    }

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
    for (int i = from; i < to; i++) 
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

void update_weights(uint32_t color0, uint32_t color1, uint32_t color2)
{
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

void switch_colors(int16_t inc)
{
    color_idx0 = (int16_t(color_idx0) + inc) & 0xff;
    uint16_t color_idx1 = (color_idx0 + 85) & 0xff;
    uint16_t color_idx2 = (color_idx0 + 171) & 0xff;

    uint32_t color0 = Wheel(byte(color_idx0));
    uint32_t color1 = Wheel(byte(color_idx1));
    uint32_t color2 = Wheel(byte(color_idx2));

    update_weights(color0, color1, color2);
}
