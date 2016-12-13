/**
 * 
 * This is code for the tie
 */

//#define LIN_OUT8 1 // use the log output function
//#define SCALE 256
//#define FFT_N 128 // set to 256 point fft
//
//#define LED_PIN 13
//#define TIME_DELAY 100
//
//#include <FFT.h> // include the library
//
//#if LOG_OUT
//#define FFT_FUNC fft_mag_log
//#define FFT_OUTPUT fft_log_out
//#elif LIN_OUT8
//#define FFT_FUNC fft_mag_lin8
//#define FFT_OUTPUT fft_lin_out8
//#else
//#error "This is invalid!!"
//#endif



#include <Adafruit_NeoPixel.h>

//#include <EEPROM.h>
//#define EEPROM_mode_address 0
//#define EEPROM_brightness_address 1
//#define EEPROM_color_address 2

#include "BluefruitConfig.h"

#include "util_funcs.hpp"

// Includes for LSM303 functionality
#include <Wire.h>
//#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

//Adafruit_LSM303 lsm;
// 


/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);

void displaySensorDetails(void)
{
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

typedef struct {
    uint8_t pin;
    uint8_t count;
} CHAIN_DATA_t;

CHAIN_DATA_t CHAIN_DATA[] = {
{ 10, 5 },
{ 11, 5 },
{ 6,  5 },
{ 12, 4 }
};

#define NUM_CHAINS (sizeof(CHAIN_DATA) / sizeof(CHAIN_DATA[0]))

uint32_t SWITCH_TIME = 500; 

#define GEA(val, exp, el) ((val >= 1<<exp) ? exp : el) 

// Parameter 1 = number of pixels in strip,  neopixel stick has 8
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_RGB     Pixels are wired for RGB bitstream
//   NEO_GRB     Pixels are wired for GRB bitstream, correct for neopixel stick
//   NEO_KHZ400  400 KHz bitstream (e.g. FLORA pixels)
//   NEO_KHZ800  800 KHz bitstream (e.g. High Density LED strip), correct for neopixel stick
Adafruit_NeoPixel g_strip[NUM_CHAINS] ; 

// The running modes
enum class VIZ_MODE : byte
{
    AMP = 0,
    FFT = 1,
    VU = 2,
    RAINBOW = 3,
    SOLID = 4,
    EQ = 5,
    LEN, 
    
    MIN = AMP,
    MAX = LEN - 1
}; 

static byte g_color_idx;                // << The current color value

static uint8_t g_brightness = 127;      // << the current brightness


sensors_vec_t prev_reading;

void setup() 
{ 
    CONFIG(); 
    
    // Change the prescaler to get a better sampling rate
//  ADCSRA &= ~0x07;
//  ADCSRA |= _BV(ADPS2) | _BV(ADPS0); // prescaler = 32
    
//  analogReference(DEFAULT);
    
    for (uint8_t i=0; i<NUM_CHAINS; i++) 
    {
        g_strip[i] = Adafruit_NeoPixel(CHAIN_DATA[i].count, CHAIN_DATA[i].pin, NEO_GRB + NEO_KHZ800); 
        g_strip[i].begin();
        g_strip[i].show();
    }
    
//  // Seed with random
//  randomSeed(analogRead(0));
    
    g_brightness = 255; 
    g_color_idx = 0; 
    
    // Setup the ble
//  ble_setup();
    
    /* Initialise the sensor */
    if(!accel.begin())
    {
      /* There was a problem detecting the ADXL345 ... check your connections */
      Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
      while(1);
    }

    /* Display some basic information on this sensor */
    displaySensorDetails();

    PRINT("Setup done");

    memset(&prev_reading, sizeof(prev_reading), 0);
}

void flow_modes(); 
void vu_mode(); 
void eq_mode(); 
void solid_mode(int idx); 
void rainbow_mode(int idx); 

void standby_mode(); 

int16_t sample(); 

void set_mode(byte mode);
void set_brightness(byte b);
void set_color(byte color_idx);

void loop() 
{
//  delay(10);
    g_color_idx++;

    if (g_color_idx > 255)
    {
        g_color_idx=0;
    }

    for (uint8_t i=0; i<NUM_CHAINS; i++) 
    {
        rainbow_mode(i);
//      g_strip[i].setBrightness(g_brightness);

        // Show the strip
        g_strip[i].show();
    }

//  int32_t maxAccel = -32000;
//  int numReadings = 0;
//
//  int currentTime = millis();
//
//  while((millis() - currentTime) < 500)
//  {
//      lsm.read();
//      int accelX = (int)lsm.accelData.x;
//      int accelY = (int)lsm.accelData.y;
//      int accelZ = (int)lsm.accelData.z;
//      Serial.print("Accel X: "); Serial.print((int)lsm.accelData.x); Serial.print(" ");
//      Serial.print("Y: "); Serial.print((int)lsm.accelData.y);       Serial.print(" ");
//      Serial.print("Z: "); Serial.println((int)lsm.accelData.z);     Serial.print(" ");
    //  Serial.print("Mag X: "); Serial.print((int)lsm.magData.x);     Serial.print(" ");
    //  Serial.print("Y: "); Serial.print((int)lsm.magData.y);         Serial.print(" ");
    //  Serial.print("Z: "); Serial.println((int)lsm.magData.z);  Serial.print(" ");

        // From https://learn.adafruit.com/lsm303-accelerometer-slash-compass-breakout/coding
//      const float Pi = 3.14159;
//      float heading = (atan2(lsm.magData.y, lsm.magData.x) * 180) / Pi;
//
//      // Normalize to 0-360
//      if (heading < 0)
//      {
//          heading = 360 + heading;
//      }
//      Serial.print("Compass Heading: ");
//      Serial.println(heading);

//      static const int TOTAL_ELEMS = 10;
//
//      static float pastValues[TOTAL_ELEMS] = {0};
//      static int idx = 0;

//        int32_t totalAccel = accelX*accelX + accelY*accelY + accelZ*accelZ;
//        maxAccel = max(totalAccel, maxAccel);
//        numReadings += 1;
//    }
//
//    Serial.print("Max accelSq: "); Serial.println(maxAccel);
////  Serial.print(","); Serial.print(totalAccelY);
////  Serial.print(","); Serial.print(totalAccelZ);
////  Serial.print(", Num Readings: "); Serial.println(numReadings);
//
//    delay(1000 - (millis() - currentTime));

    /* Get a new sensor event */
    sensors_event_t event;
    accel.getEvent(&event);

    /* Display the results (acceleration is measured in m/s^2) */
    Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
    Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
    Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");

    sensors_vec_t delta;

    for(int i=0; i<3; i++) {
        delta.v[i] = prev_reading.v[i] - event.acceleration.v[i];
        prev_reading.v[i] = event.acceleration.v[i];
    }
    Serial.print("DeltaX: "); Serial.print(delta.x); Serial.print("  ");
    Serial.print("DeltaY: "); Serial.print(delta.y); Serial.print("  ");
    Serial.print("DeltaZ: "); Serial.print(delta.z); Serial.print("  ");Serial.println("m/s^2 ");

//
///* Note: You can also get the raw (non unified values) for */
///* the last data sample as follows. The .getEvent call populates */
///* the raw values used below. */
////Serial.print("X Raw: "); Serial.print(accel.raw.x); Serial.print("  ");
////Serial.print("Y Raw: "); Serial.print(accel.raw.y); Serial.print("  ");
////Serial.print("Z Raw: "); Serial.print(accel.raw.z); Serial.println("");
//
///* Delay before the next sample */
    delay(100);
}

//void standby_mode()
//{
//    // sleep for a second
//    delay(1000);
//
//    int16_t total_amp = sample();
//
//    if (abs(total_amp) > STANDBY_TRIGGER_AMOUNT)
//    {
//        g_standby_mode = false;
//        g_prev_standby_time = millis();
//    }
//}
//
//void regular_mode()
//{
//    int16_t total_amp = sample();
//
//    uint16_t new_time = millis();
//
//    if (abs(total_amp) > STANDBY_TRIGGER_AMOUNT)
//    {
//        g_prev_standby_time = new_time;
//    } else if ((new_time - g_prev_standby_time) > STANDBY_MODE_TIME)
//    {
//        g_standby_mode = true;
//        // Show the strip in standby mode
//        for (uint16_t i = 0; i < PIXEL_COUNT; i++)
//        {
//            strip.setPixelColor(i, STANDBY_MODE_COLOR);
//        }
//        strip.setBrightness(0);
//        strip.show();
//        return;
//    }
//
//    switch (g_current_mode)
//    {
//    case VIZ_MODE::FFT:
//    case VIZ_MODE::AMP:
//        flow_modes();
//        break;
//    case VIZ_MODE::VU:
//        vu_mode();
//        break;
//    case VIZ_MODE::EQ:
//        eq_mode();
//        break;
//    case VIZ_MODE::SOLID:
//        solid_mode();
//        break;
//    case VIZ_MODE::RAINBOW:
//        rainbow_mode();
//        break;
//    default:
//        break;
//    }
//
//    strip.setBrightness(g_brightness);
//
//    // Show the strip
//    strip.show();
//
//    // Switch every SWITCH_TIME
//    if ((new_time - g_prev_time) > SWITCH_TIME)
//    {
//        switch_colors();
//
//        g_prev_time = new_time;
//
//        //      PRINT(F("total_amp = "));
//        //      PRINT(total_amp);
//    }
//
//    if (ble_update())
//    {
//        // if we receive a color, goto solid mode
//        uint32_t color;
//        if (ble_get_color(color))
//        {
//            set_mode(byte(VIZ_MODE::SOLID));
//            set_color(color);
//        }
//
//        uint8_t button;
//        bool pressed;
//        if (ble_get_button(button, pressed))
//        {
//            // React on the release only
//            if (!pressed)
//            {
//                if (1 <= button && button <= 4)
//                {
//                    VIZ_MODE new_mode = VIZ_MODE(byte(button - 1) % byte(VIZ_MODE::LEN));
//                    set_mode(byte(new_mode));
//
//                    switch (g_current_mode)
//                    {
//                    case(VIZ_MODE::EQ):
//                        eq_setup_mode();
//                        break;
//                    default:
//                        break;
//                    }
//                }
//                else if (button == 5) // up
//                {
//                    if (g_brightness < (255 - BRIGHTNESS_INC))
//                    {
//                        set_brightness(g_brightness + BRIGHTNESS_INC);
//                    }
//                }
//                else if (button == 6) // down
//                {
//                    if (g_brightness > BRIGHTNESS_INC)
//                    {
//                        set_brightness(g_brightness - BRIGHTNESS_INC);
//                    }
//                }
//                else if (button == 7) // left
//                {
//                    switch_colors(-16);
//                    g_prev_time = new_time;
//                }
//                else if (button == 8) // right
//                {
//                    switch_colors(16);
//                    g_prev_time = new_time;
//                }
//            }
//        }
//    }
//}
//
//
//void set_mode(byte mode)
//{
//    g_current_mode = VIZ_MODE(mode);
//    // update the current_mode in the EEPROM
//    EEPROM.update(EEPROM_mode_address, mode);
//}
//
//void set_brightness(byte b)
//{
//    g_brightness = b;
//    EEPROM.update(EEPROM_brightness_address, g_brightness);
//}
//
//void set_color(byte color_idx)
//{
//    g_color_idx = color_idx;
//    EEPROM.update(EEPROM_color_address, g_color_idx);
//}
//
//
//uint32_t fft_to_color();
//uint32_t amp_to_color();
//
//void flow_modes()
//{
//    uint32_t color;
//
//    if (g_current_mode == VIZ_MODE::FFT)
//    {
//        color = fft_to_color();
//    }
//    else
//    {
//        color = amp_to_color();
//    }
//
//    static uint32_t prev_colors[PIXEL_COUNT] = { 0 };
//    // move each color down
//    for (uint16_t i = PIXEL_COUNT - 1; i > 0; i--)
//    {
//        prev_colors[i] = prev_colors[i - 1];
//    }
//    prev_colors[0] = color;
//    // update the pixel colors
//    for (uint16_t i = 0; i < PIXEL_COUNT; i++)
//    {
//        strip.setPixelColor(i, prev_colors[i]);
//    }
//
//#if 0 // DEBUG
//    //      Serial.write(255); // send a start byte
//    //      Serial.write(bins, 3); // send out the data
//    //      Serial.flush();
//
//    //      delay(200);
//#endif
//}
//
//// Shared static functions and data
////
//#define INPUT_FLOOR 19 //Lower range of analogRead input
//#define INPUT_CEILING 600 //Max range of analogRead input, the lower the value the more sensitive (1023 = max)
//
//float fscale(float originalMin, float originalMax, float newBegin, float
//             newEnd, float inputValue, float curve);
////
////// start Mode::FFT functionality
////
//
//int16_t sample()
//{
//    int16_t total_amp = 0;
//    for (int i = 0; i < FFT_N * 2; i += 2) // save FFT_N samples
//    {
//        static const int16_t noiseThreshold = 6;
//        int16_t k = analogRead(MIC_PIN);
//        k = ((k > (512 - noiseThreshold)) &&
//             (k < (512 + noiseThreshold))) ? 0 :
//                                             k - 512; // Sign-convert for FFT; -512 to +511
//        k <<= 6; // form into a 16b signed int
//        fft_input[i] = k; // put real data into even bins
//        fft_input[i + 1] = 0; // set odd bins to 0
//
//        total_amp += k;
//    }
//
//    return total_amp;
//}
//
//// Returns an array of uint8_t of size FFT_N/2
//uint8_t* calc_fft_input()
//{
//    fft_window(); // window the data for better frequency response
//    fft_reorder(); // reorder the data before doing the fft
//    fft_run(); // process the data in the fft
//    FFT_FUNC(); // take the output of the fft
//
//    return FFT_OUTPUT;
//}
//
//uint32_t fft_to_color()
//{
//    uint8_t *output = calc_fft_input();
//
//    static uint16_t bins[3] = { 0 };
//
//    for (int i = 0; i < 3; i++)
//    {
//        bins[i] = 0;
//    }
//
//    // For the Frequency modes (FFT and EQ), use this as a rough division between low, mid and high bands
//    static const uint8_t BAND_END[3] = { 9, 27, 63 };
//
//    uint8_t current_bin = 0;
//    for (int i = 0; i < FFT_N / 2; i++)
//    {
//        uint8_t val = output[i];
//
//        bins[current_bin] += GEA(val, 7, GEA(val, 6, GEA(val, 5, GEA(val, 4, GEA(val, 3, GEA(val, 2, GEA(val, 1, 0)))))));
//
//        // This is a hacky way of splitting up the frequency domains
//        if (i >= BAND_END[current_bin])
//        {
//            current_bin++;
//        }
//    }
//
//    uint16_t r = (bins[0] * color_weights[0] + bins[1] * color_weights[1] + bins[2] * color_weights[2]) >> (8 - 2);
//    uint16_t g = (bins[0] * color_weights[3] + bins[1] * color_weights[4] + bins[2] * color_weights[5]) >> (8 - 2);
//    uint16_t b = (bins[0] * color_weights[6] + bins[1] * color_weights[7] + bins[2] * color_weights[8]) >> (8 - 2);
//
//    uint32_t new_color = Adafruit_NeoPixel::Color(uint8_t(r), uint8_t(g), uint8_t(b));
//    return new_color;
//}
//
//// start Mode::AMP functionality
//
//// From timings, I derived that this is the min and max for the
//// particular mic setup. Do a linear scale from these values to 0-255
//uint32_t amp_to_color()
//{
//    // Linear amplitude display
//    int16_t max_val, min_val;
//    max_val = min_val = fft_input[0];
//    for (int i = 1; i < FFT_N; i++)
//    {
//        int16_t val = fft_input[i * 2];
//        if (val < min_val)
//        {min_val = val; }
//        else if (val > max_val)
//        {max_val = val; }
//    }
//    uint16_t w = max_val - min_val;
//
//    //Scale the input logarithmically instead of linearly
//    w = fscale(1200, 32000, 0, 255, w, 2);
//
//#if 0 // DEBUG
//    uint16_t new_time = millis();
//
//    // Switch every SWITCH_TIME
//    if ((new_time - prev_time) > (SWITCH_TIME-10))
//    {
//        Serial.print("AMP mode: value = "); Serial.println(w);
//    }
//#endif
//
//    uint16_t r;
//    uint16_t g;
//    uint16_t b;
//
//    if (w > 150)
//    {
//        r = (w * color_weights[0]) >> 8;
//        g = (w * color_weights[1]) >> 8;
//        b = (w * color_weights[2]) >> 8;
//    }
//    else if (w > 45)
//    {
//        r = (w * color_weights[3]) >> 8;
//        g = (w * color_weights[4]) >> 8;
//        b = (w * color_weights[5]) >> 8;
//    }
//    else
//    {
//        r = (w * color_weights[6]) >> 8;
//        g = (w * color_weights[7]) >> 8;
//        b = (w * color_weights[8]) >> 8;
//    }
//
//    return Adafruit_NeoPixel::Color(uint8_t(r), uint8_t(g), uint8_t(b));
//}
//
//// start VIZ_MODE::VU code
//// NOTE: taken from Adafruit's original tie code
//#define SAMPLE_WINDOW   10  // Sample window for average level
//#define PEAK_HANG 24 //Time of pause before peak dot falls
//#define PEAK_FALL 4 //Rate of falling peak dot
//
//void drawLine(uint8_t from, uint8_t to, uint32_t c);
//uint32_t Wheel(byte WheelPos);
//
//void vu_mode()
//{
//    int16_t max_val, min_val;
//    max_val = min_val = fft_input[0];
//    for (int i = 1; i < FFT_N; i++)
//    {
//        int16_t val = fft_input[i * 2];
//        if (val < min_val)
//        {min_val = val; }
//        else if (val > max_val)
//        {max_val = val; }
//    }
//    uint16_t w = max_val - min_val;
//    w >>= (6 - 2);
//
//    //Scale the input logarithmically instead of linearly
//    uint16_t c = fscale(INPUT_FLOOR, INPUT_CEILING, 0, strip.numPixels(), w, 2);
//
//    static byte peak = 16;      // Peak level of column; used for falling dots
//    static byte dotCount = 0;  //Frame counter for peak dot
//    static byte dotHangCount = 0; //Frame counter for holding peak dot
//
//    if (c >= peak)
//    {
//        peak = c;        // Keep dot on top
//        dotHangCount = 0;    // make the dot hang before falling
//    }
//
//    const uint8_t bottom_color = 30;
//    const uint8_t top_color = 150;
//
//    //Fill the strip with rainbow gradient
//    for (uint16_t i = 0; i < c; i++)
//    {
//        uint32_t color = Wheel(map(i, 0, strip.numPixels() - 1, bottom_color, top_color) + g_color_idx);
//        strip.setPixelColor(i, color);
//    }
//    // Fill the rest with black
//    drawLine(c, strip.numPixels(), strip.Color(0, 0, 0));
//
//    // Set the peak dot to match the rainbow gradient
//    uint32_t color = Wheel(map(peak, 0, strip.numPixels() - 1, bottom_color, top_color) + g_color_idx);
//    strip.setPixelColor(peak, color);
//
//    // Frame based peak dot animation
//    if (dotHangCount > PEAK_HANG) //Peak pause length
//    {
//        if (++dotCount >= PEAK_FALL) //Fall rate
//        {
//            peak--;
//            dotCount = 0;
//        }
//    }
//    else
//    {
//        dotHangCount++;
//    }
//}
//
//static uint16_t eq_pixels[PIXEL_COUNT];
//
//void eq_setup_mode()
//{
//    // Clear the pixel values
//    for (int i = 0; i < PIXEL_COUNT; i++)
//    {
//        eq_pixels[i] = 0;
//    }
//}
//
//
//void eq_mode()
//{
//    uint8_t *output = calc_fft_input();
//
//    // This maps the bands to the pixels
//#if PIXEL_COUNT == 60
//    static const uint8_t FREQ_PER_PIXEL[3] = { 1, 1, 1 };
//    static const uint8_t BAND_PIXEL_END[3] = { 19, 51, 59 };
//#else
//    // THIS is NOT tuned YET!
//    static const uint8_t FREQ_PER_PIXEL[3] = {8, 8, 8};
//    static const uint8_t BAND_PIXEL_END[3] = {3, 13, 15};
//#endif
//
//    uint8_t band = 0;
//    uint8_t freq = 0;
//    for (uint8_t current_pixel = 0; current_pixel < PIXEL_COUNT; current_pixel++)
//    {
//        eq_pixels[current_pixel] = (uint32_t(eq_pixels[current_pixel]) * 15) / 16;
//
//        for (int j = 0; j < FREQ_PER_PIXEL[band]; j++)
//        {
//            uint8_t val = output[freq];
//            eq_pixels[current_pixel] += GEA(val, 7, GEA(val, 6, GEA(val, 5, GEA(val, 4, GEA(val, 3, GEA(val, 2, GEA(val, 1, 0)))))));
//            freq++;
//        }
//
//        if (current_pixel >= BAND_PIXEL_END[band])
//        {
//            band++;
//        }
//    }
//
//    band = 0;
//    for (uint8_t i = 0; i < PIXEL_COUNT; i++)
//    {
//        uint16_t val = eq_pixels[i];
//
//        uint16_t r = (val * color_weights[band * 3 + 0]) >> (8 - 5);
//        uint16_t g = (val * color_weights[band * 3 + 1]) >> (8 - 5);
//        uint16_t b = (val * color_weights[band * 3 + 2]) >> (8 - 5);
//        strip.setPixelColor(i, uint8_t(r), uint8_t(g), uint8_t(b));
//
//        if (i >= BAND_PIXEL_END[band])
//        {
//            band++;
//        }
//    }
//
//#if 0 // DEBUG
//    Serial.write(255);
//
//    //  static uint8_t data[PIXEL_COUNT];
//    //  for (uint8_t i=0; i<PIXEL_COUNT; i++)
//    //  {
//    //      uint8_t val = bins[i];
//    //      val = val==255? 254: val;
//    //      data[i] = val;
//    //  }
//    Serial.write(output, FFT_N/2);
//#endif
//
//}

void rainbow_mode(int index)
{
    // solid rainbow color
    for (uint16_t i=0; i < g_strip[index].numPixels(); i++) {
        uint32_t c = Wheel(index, i + g_color_idx );
        g_strip[index].setPixelColor(i, c);
    }
}

void solid_mode(int index)
{
    uint32_t c = Wheel(index, g_color_idx );
    for (uint16_t i=0; i < g_strip[index].numPixels(); i++) {
        g_strip[index].setPixelColor(i, c);
    }
}


//Used to draw a line between two points of a given color
//void drawLine(uint8_t from, uint8_t to, uint32_t c)
//{
//    uint8_t fromTemp;
//    if (from > to)
//    {
//        fromTemp = from;
//        from = to;
//        to = fromTemp;
//    }
//    for (int i = from; i < to; i++)
//    {
//        strip.setPixelColor(i, c);
//    }
//}
//
//
//float fscale(float originalMin, float originalMax, float newBegin, float
//             newEnd, float inputValue, float curve)
//{
//
//    float OriginalRange = 0;
//    float NewRange = 0;
//    float zeroRefCurVal = 0;
//    float normalizedCurVal = 0;
//    float rangedValue = 0;
//    boolean invFlag = 0;
//
//
//    // condition curve parameter
//    // limit range
//
//    if (curve > 10) curve = 10;
//    if (curve < -10) curve = -10;
//
//    curve = (curve * -.1); // - invert and scale - this seems more intuitive - postive numbers give more weight to high end on output
//    curve = pow(10, curve); // convert linear scale into lograthimic exponent for other pow function
//
//    /*
//     Serial.println(curve * 100, DEC);   // multply by 100 to preserve resolution
//     Serial.println();
//     */
//
//    // Check for out of range inputValues
//    if (inputValue < originalMin)
//    {
//        inputValue = originalMin;
//    }
//    if (inputValue > originalMax)
//    {
//        inputValue = originalMax;
//    }
//
//    // Zero Refference the values
//    OriginalRange = originalMax - originalMin;
//
//    if (newEnd > newBegin)
//    {
//        NewRange = newEnd - newBegin;
//    }
//    else
//    {
//        NewRange = newBegin - newEnd;
//        invFlag = 1;
//    }
//
//    zeroRefCurVal = inputValue - originalMin;
//    normalizedCurVal  =  zeroRefCurVal / OriginalRange;   // normalize to 0 - 1 float
//
//    // Check for originalMin > originalMax  - the math for all other cases i.e. negative numbers seems to work out fine
//    if (originalMin > originalMax)
//    {
//        return 0;
//    }
//
//    if (invFlag == 0)
//    {
//        rangedValue =  (pow(normalizedCurVal, curve) * NewRange) + newBegin;
//    }
//    else     // invert the ranges
//    {
//        rangedValue =  newBegin - (pow(normalizedCurVal, curve) * NewRange);
//    }
//
//    return rangedValue;
//}


// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(int index, byte WheelPos) 
{ 
    if (WheelPos < 85) 
    {
        return g_strip[index].Color(WheelPos * 3, 255 - WheelPos * 3, 0);
    } 
    else if (WheelPos < 170) 
    {
        WheelPos -= 85; 
        return g_strip[index].Color(255 - WheelPos * 3, 0, WheelPos * 3);
    } 
    else 
    {
        WheelPos -= 170; 
        return g_strip[index].Color(0, WheelPos * 3, 255 - WheelPos * 3);
    }
}

//void update_weights(uint32_t color0, uint32_t color1, uint32_t color2)
//{
//    color_weights[0] = (color0 >> 16) & 0xff;
//    color_weights[1] = (color0 >> 8) & 0xff;
//    color_weights[2] = (color0 >> 0) & 0xff;
//    color_weights[3] = (color1 >> 16) & 0xff;
//    color_weights[4] = (color1 >> 8) & 0xff;
//    color_weights[5] = (color1 >> 0) & 0xff;
//    color_weights[6] = (color2 >> 16) & 0xff;
//    color_weights[7] = (color2 >> 8) & 0xff;
//    color_weights[8] = (color2 >> 0) & 0xff;
//}

//void switch_colors(int16_t inc)
//{
//    set_color(byte(inc + g_color_idx));
//    // NOTE: this may not work right!!
//    byte color_idx1 = (g_color_idx + 85);
//    byte color_idx2 = (g_color_idx + 171);
//
//    uint32_t color0 = Wheel(g_color_idx);
//    uint32_t color1 = Wheel(color_idx1);
//    uint32_t color2 = Wheel(color_idx2);
//
//    update_weights(color0, color1, color2);
//}
