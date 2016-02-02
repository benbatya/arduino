
#define LOG_OUT 1 // use the log output function
#define FFT_N 256 // set to 256 point fft

#define LED_PIN 13
#define TIME_DELAY 100

#include <FFT.h> // include the library

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

    LEN
};

static MODE current_mode = MODE::FFT;

static int32_t prev_time;
static uint8_t color_weights[9];

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

    if (EEPROM.length() > 0)  // Only try to use EEPROM if there is some on the board
    {
        static const int EEPROM_address = 0;

        // Get the previous mode. If it's valid, set current_mode the next mode % MODE::LEN
        MODE value = MODE(EEPROM.read(EEPROM_address));
        switch (value) 
        {
        case MODE::AMP:
            current_mode = MODE::FFT;
            break;
        case MODE::FFT:
            current_mode = MODE::VU;
            break;
        case MODE::VU:
            current_mode = MODE::AMP;
            break;
        default:
            break;
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
    default:
        break;
    }

    strip.show();
    delay(2000);

    prev_time = millis();

    // Init the color weights
    for (int i=0; i<9; i++) 
    {
        color_weights[i] = random(256);
    }
}

void flow_modes();
void vu_mode();

void loop() 
{   
    switch (current_mode) 
    {
    case MODE::FFT:
    case MODE::AMP:
        flow_modes(); 
        break;
    case MODE::VU:
        vu_mode();
        break;
    default:
        break;
    }
}


uint32_t fft_to_color();
uint32_t amp_to_color();

void flow_modes()
{
    uint32_t color;

    switch (current_mode) 
    {
    case MODE::FFT:
        color = fft_to_color();
        break;
    case MODE::AMP:
        color = amp_to_color();
        break;
    default:
        return;
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

// start Mode::FFT functionality

// This is low-level noise that's subtracted from each FFT output column:
static const uint8_t PROGMEM noise[128]=
  // This is low-level noise that's subtracted from each FFT output column:
{  100,90,80,80,80,80,80,80,80,80,80,70,70,70,70,70,
    70,70,70,70,70,70,70,70,70,70,80,70,70,70,70,70,
    96,106,90,70,70,70,70,70,70,70,70,70,70,70,70,70,
    70,70,70,70,70,70,70,70,70,70,80,86,80,70,70,70,
    70,105,110,95,70,70,70,70,70,70,70,70,70,70,70,70,
    70,70,70,70,70,70,70,70,70,70,70,90,90,80,70,70,
    70,70,105,105,90,70,70,70,70,70,70,70,70,70,70,70,
    70,70,70,70,70,70,70,70,70,70,70,85,95,100,70,70};

uint32_t fft_to_color()
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
    fft_mag_log(); // take the output of the fft
    
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
    return new_color;
}

// start Mode::AMP functionality

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
uint32_t amp_to_color()
{
    // Linear amplitude display
    uint16_t amp = amplitude();
//  int16_t amp = analogRead(MIC_PIN);
//  amp -= 512; // Remove the DC offset
//  amp <<= 6;  // make it expand to fill the full signed 16 bit range

    // Used for auto scaling
    static const uint8_t NUM_PREV_AMPS = 20;
    static uint16_t prev_amp[NUM_PREV_AMPS] = {0};

    for (int i=NUM_PREV_AMPS-1; i>0; i--) 
    {
        prev_amp[i] = prev_amp[i-1];
    }
    prev_amp[0] = amp;

    uint16_t min_amp, max_amp;
    min_amp = max_amp = amp;
    for (int i=1; i<NUM_PREV_AMPS; i++) 
    {
        uint16_t amp = prev_amp[i];
        // Update the min and max
        if (min_amp > amp)
        {
            min_amp = amp;
        }
        if (max_amp < amp)
        {
            max_amp = amp;
        }
    }

//  Serial.print("value = "); Serial.print(amp);
//  Serial.print(" ,max = "); Serial.print(max_amp);
//  Serial.print(" ,min = "); Serial.println(min_amp);

    uint32_t temp = uint32_t(amp - min_amp) * 0xff / (max_amp - min_amp);
    // Run abs on temp
//  temp = (temp < 0) ? -temp: temp;

    static const uint16_t MIN_AMP = 30;

    if (temp < MIN_AMP) 
    {
        temp = 0;
    }

    uint16_t w = uint16_t(temp);
//  uint16_t r = (w*color_weights[0] + w*color_weights[1] + w*color_weights[2]) / 256;
//  uint16_t g = (w*color_weights[3] + w*color_weights[4] + w*color_weights[5]) / 256;
//  uint16_t b = (w*color_weights[6] + w*color_weights[7] + w*color_weights[8]) / 256;

//  Serial.print("final = "); Serial.println(g);

//  return Adafruit_NeoPixel::Color(uint8_t(r), uint8_t(g), uint8_t(b));
    return Adafruit_NeoPixel::Color(uint8_t(w), uint8_t(w), uint8_t(w));
}

// start MODE::VU code
// NOTE: taken from Adafruit's original tie code
#define SAMPLE_WINDOW   10  // Sample window for average level
#define PEAK_HANG 24 //Time of pause before peak dot falls
#define PEAK_FALL 4 //Rate of falling peak dot
#define INPUT_FLOOR 22 //Lower range of analogRead input
#define INPUT_CEILING 300 //Max range of analogRead input, the lower the value the more sensitive (1023 = max)

byte peak = 16;      // Peak level of column; used for falling dots
//unsigned int sample;

byte dotCount = 0;  //Frame counter for peak dot
byte dotHangCount = 0; //Frame counter for holding peak dot

void drawLine(uint8_t from, uint8_t to, uint32_t c);
float fscale(float originalMin, float originalMax, float newBegin, float
             newEnd, float inputValue, float curve);

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
    
    strip.show(); 
    
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
