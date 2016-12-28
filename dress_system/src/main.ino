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

#include <math.h>

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

#define MAX_PIXELS_IN_CHAIN 5

CHAIN_DATA_t CHAIN_DATA[] = {
{ 6,  5 },
{ 10, 5 },
{ 11, 5 },
{ 12, 4 }
};

#define NUM_CHAINS (sizeof(CHAIN_DATA) / sizeof(CHAIN_DATA[0]))

#define BACK_CHAIN 0

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
    STANDBY = 0,
    RAINBOW = 1,
    TWINKLE = 2,
//  WATERFALL = 3,
    DIRECTION = 3,
    LEN, 
    
    MIN = STANDBY,
    MAX = LEN - 1
}; 

static VIZ_MODE g_mode = VIZ_MODE::STANDBY;

static byte g_color_idx;                // << The current color value


sensors_vec_t prev_reading;

void get_delta_acceleration(sensors_vec_t* delta)
{
    /* Get a new sensor event */
    sensors_event_t event;
    accel.getEvent(&event);

    /* Display the results (acceleration is measured in m/s^2) */
//  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
//  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
//  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");

    for(int i=0; i<3; i++) {
        float val = prev_reading.v[i] - event.acceleration.v[i];
        prev_reading.v[i] = event.acceleration.v[i];

        val = val < 1.0f ? 1.0f : val;

        val = log(val);

        val = val > 4.0f ? 4.0f : val;

        delta->v[i] = val; 
//      Serial.print(delta_accel.v[i]); Serial.print(", ");
    }
//  Serial.println("");

//  Serial.print("DeltaX: "); Serial.print(delta.x); Serial.print("  ");
//  Serial.print("DeltaY: "); Serial.print(delta.y); Serial.print("  ");
//  Serial.print("DeltaZ: "); Serial.print(delta.z); Serial.print("  ");Serial.println("m/s^2 ");

}

#define STANDBY_MODE_COLOR 0 
void start_standby_mode()
{
    for (uint8_t i=0; i<NUM_CHAINS; i++) 
    {
        Adafruit_NeoPixel& strip = g_strip[i];
        // Show the strip in standby mode
        for (uint16_t j = 0; j < strip.numPixels(); j++)
        {
            strip.setPixelColor(j, STANDBY_MODE_COLOR);
        }
//      strip.setBrightness(g_brightness);
        strip.show();
    }
}

void run_standby_mode()
{
    // sleep for a second
    delay(500);
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) 
{ 
    if (WheelPos < 85) 
    {
        return Adafruit_NeoPixel::Color(WheelPos * 3, 255 - WheelPos * 3, 0);
    } 
    else if (WheelPos < 170) 
    {
        WheelPos -= 85; 
        return Adafruit_NeoPixel::Color(255 - WheelPos * 3, 0, WheelPos * 3);
    } 
    else 
    {
        WheelPos -= 170; 
        return Adafruit_NeoPixel::Color(0, WheelPos * 3, 255 - WheelPos * 3);
    }
}

static uint8_t color_weights[9]; 

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
    set_color(byte(inc + g_color_idx)); 
    byte color_idx1 = (g_color_idx + 85); 
    byte color_idx2 = (g_color_idx + 171); 
    
    uint32_t color0 = Wheel(g_color_idx); 
    uint32_t color1 = Wheel(color_idx1); 
    uint32_t color2 = Wheel(color_idx2); 
    
    update_weights(color0, color1, color2);
}

void update_colors()
{
    static uint8_t count = 0;
    if (count < 30) 
    {
        count = 0;
        switch_colors(10);
    }
    count += 1;
}

void set_color(byte color_idx)
{
    g_color_idx = color_idx;
//  EEPROM.update(EEPROM_color_address, g_color_idx);
}

void rainbow_mode(Adafruit_NeoPixel& strip)
{
    for(uint8_t i=0; i<strip.numPixels(); i++) {
        uint32_t c = Wheel( i + g_color_idx );
        strip.setPixelColor(i, c);
    }
}

void start_rainbow_mode()
{
    g_color_idx=0;
}

void run_rainbow_mode()
{
    if (g_color_idx > 255)
    {
        g_color_idx=0;
    }

    for (uint8_t i=0; i<NUM_CHAINS; i++) 
    {
        Adafruit_NeoPixel& strip = g_strip[i];
        rainbow_mode(strip);

        // Show the strip
        strip.show();
    }

    g_color_idx++;

    delay(100);
}

enum class TWINKLE_STATE : byte
{
    ACENDING = 0,
    DECENDING = 1,
    SLEEP = 2,
    LEN, 
    
    MIN = ACENDING,
    MAX = LEN - 1
}; 

typedef struct TWINKLE {
    TWINKLE_STATE state;
    // Length of time (in ms) to wait until starting to change color
    uint8_t sleep_delay;
    // positive means that the pixel is going towards its max brightness
    // negitive means that it is going towards zero
    uint8_t speed;
    // the set color
    uint8_t target_color[3];
    // The maximum brightness to go to
    uint8_t percentage;
} TWINKLE_t;

TWINKLE_t g_twinkle_data[NUM_CHAINS][MAX_PIXELS_IN_CHAIN];
unsigned long g_prev_time;

void gen_twinkle_data(struct TWINKLE& twinkle_data, sensors_vec_t& delta)
{
    twinkle_data.state = TWINKLE_STATE::ACENDING;
    twinkle_data.sleep_delay = uint8_t(random(60));
    twinkle_data.speed = uint8_t((random(16)+4));
    uint16_t bins[3];
    for (uint8_t k=0; k<3; k++)
    {
        bins[k] = delta.v[k] * 64;
    }
    twinkle_data.target_color[0] = (bins[0] * color_weights[0] + bins[1] * color_weights[1] + bins[2] * color_weights[2]) >> 8; 
    twinkle_data.target_color[1] = (bins[0] * color_weights[3] + bins[1] * color_weights[4] + bins[2] * color_weights[5]) >> 8; 
    twinkle_data.target_color[2] = (bins[0] * color_weights[6] + bins[1] * color_weights[7] + bins[2] * color_weights[8]) >> 8; 

    twinkle_data.percentage = 20;
}

void start_twinkle_mode()
{
    sensors_vec_t delta_accel;
    get_delta_acceleration(&delta_accel);

    memset(g_twinkle_data, 0, sizeof(g_twinkle_data));
    // Set the twinkle data to randomish values
    for (uint8_t i=0; i<NUM_CHAINS; i++)
    {
        Adafruit_NeoPixel& strip = g_strip[i];
        for (uint8_t j=0; j<strip.numPixels(); j++)
        {
            TWINKLE_t& twinkle_data = g_twinkle_data[i][j];
            gen_twinkle_data(twinkle_data, delta_accel);
        }
    }

    g_prev_time = millis();
}

void run_twinkle_mode()
{
    unsigned long new_time = millis();
    // The counter has looped so skip this iteration
    if (new_time < g_prev_time)
    {
        g_prev_time = new_time;
        return;
    }
    unsigned long delta_time = new_time - g_prev_time;
    g_prev_time = new_time;

    update_colors();

    sensors_vec_t delta_accel;
    get_delta_acceleration(&delta_accel);
//  Serial.print("delta_accel = ");
//  Serial.print(delta_accel.v[0]); Serial.print(", ");
//  Serial.print(delta_accel.v[1]); Serial.print(", ");
//  Serial.println(delta_accel.v[2]);

//  PRINT("run_twinkle_mode: delta =");
//  PRINT(delta_time);

    // update each pixel
    for (uint8_t i=0; i<NUM_CHAINS; i++)
    {
        Adafruit_NeoPixel& strip = g_strip[i];
        for (uint8_t j=0; j<strip.numPixels(); j++)
        {
            TWINKLE_t& twinkle_data = g_twinkle_data[i][j];
            byte color[3];

//          if (!i && !j)
//          {
//              Serial.print("Pixel ("); Serial.print(i); Serial.print(", "); Serial.print(j); Serial.print("): state="); Serial.println(twinkle_data.state);
//          }
//          Serial.print("")
            switch (twinkle_data.state)
            {
            case TWINKLE_STATE::ACENDING: // The pixel is moving towards its target color
                
                for (byte i=0; i<3; i++)
                {
                    uint16_t temp = (uint16_t(twinkle_data.percentage-20) * twinkle_data.target_color[i]) / 100;
                    color[i] = uint8_t(temp);
                }
//              if (!i && !j)
//              {
//                  Serial.print("percentage="); Serial.println(twinkle_data.percentage);
//                  Serial.print("color=("); Serial.print(color[0]); Serial.print(","); Serial.print(color[1]); Serial.print(","); Serial.print(color[2]); Serial.println(")");
//              }
                strip.setPixelColor(j, color[0], color[1], color[2]);
                twinkle_data.percentage += twinkle_data.speed;
                if (twinkle_data.percentage >= 120)
                {
                    twinkle_data.percentage = 120;
                    twinkle_data.state = TWINKLE_STATE::DECENDING;
                }
                break;
            case TWINKLE_STATE::DECENDING:
                for (byte i=0; i<3; i++)
                {
                    uint16_t temp = (uint16_t(twinkle_data.percentage-20) * twinkle_data.target_color[i]) / 100;
                    color[i] = uint8_t(temp);
                }
//              if (!i && !j)
//              {
//                  Serial.print("percentage="); Serial.println(twinkle_data.percentage);
//                  Serial.print("color=("); Serial.print(color[0]); Serial.print(","); Serial.print(color[1]); Serial.print(","); Serial.print(color[2]); Serial.println(")");
//              }
                strip.setPixelColor(j, color[0], color[1], color[2]);
                twinkle_data.percentage -= twinkle_data.speed;
                if (twinkle_data.percentage <= 20)
                {
                    twinkle_data.state = TWINKLE_STATE::SLEEP;
                }
                break;
            case TWINKLE_STATE::SLEEP: // The pixel is sleeping
//              if (!i && !j)
//              {
//                  Serial.print("sleep_delay="); Serial.println(twinkle_data.sleep_delay);
//              }
                if (twinkle_data.sleep_delay <= delta_time)
                {
                    gen_twinkle_data(twinkle_data, delta_accel);
                } else {
                    twinkle_data.sleep_delay -= delta_time;
                }
                strip.setPixelColor(j, 0);
                break;
            default:
                break;
            }
        }

        // Show the strip
        strip.show();
    }
    delay(25);
}

//void run_direction_mode()
//{
//    sensors_vec_t delta_accel;
//    get_delta_acceleration(&delta_accel);
//}

#define BRIGHTNESS_INC      8

static uint8_t g_brightness = 127;      // << the current brightness

void set_brightness(byte b)
{
    g_brightness = b;
    for (uint8_t i=0; i<NUM_CHAINS; i++) 
    {
        Adafruit_NeoPixel& strip = g_strip[i];
        strip.setBrightness(g_brightness);
    }
}

void handle_ble()
{
    if (ble_update()) 
    {
        // if we receive a color, goto solid mode
        uint32_t color; 
        if (ble_get_color(color)) 
        {
            set_color(color);
        } 
        
        uint8_t button; 
        bool pressed; 
        if (ble_get_button(button, pressed)) 
        {
            // React on the release only
            if (!pressed) 
            {
                if (1 <= button && button <= 4) 
                {           
                    Serial.print("Button pressed = "); Serial.println(button);
                    g_mode = VIZ_MODE(byte(button - 1) % byte(VIZ_MODE::LEN));

                    // Setup the new mode
                    switch (g_mode)
                    {
                    case VIZ_MODE::STANDBY: start_standby_mode(); break;
                    case VIZ_MODE::RAINBOW: start_rainbow_mode(); break;
                    case VIZ_MODE::TWINKLE: start_twinkle_mode(); break;
//                  case VIZ_MODE::DIRECTION: start_twinkle_mode(); break;
                    default:
                        break;
                    }
                }
                else if (button == 5) // up
                {
                    if (g_brightness < (255 - BRIGHTNESS_INC)) 
                    {
                        set_brightness(g_brightness + BRIGHTNESS_INC); 
                    }
                } 
                else if (button == 6) // down
                {
                    if (g_brightness > BRIGHTNESS_INC) 
                    {
                        set_brightness(g_brightness - BRIGHTNESS_INC);
                    }
                }  
            }
        }
    }
}


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
    
    // Seed with random
    randomSeed(analogRead(0));
    
    set_brightness(g_brightness);

    // Setup the ble
    ble_setup();
    
    /* Initialise the sensor */
    if(!accel.begin())
    {
      /* There was a problem detecting the ADXL345 ... check your connections */
      Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
      while(1);
    }

    /* Display some basic information on this sensor */
    displaySensorDetails();

//  PRINT("Setup done");

    memset(&prev_reading, sizeof(prev_reading), 0);
}


void loop() 
{
    handle_ble();

    switch (g_mode) 
    {
    case VIZ_MODE::STANDBY: run_standby_mode(); break;
    case VIZ_MODE::RAINBOW: run_rainbow_mode(); break;
    case VIZ_MODE::TWINKLE: run_twinkle_mode(); break;
//  case VIZ_MODE::DIRECTION: run_direction_mode(); break;
    default:
        break;

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

  
//
///* Note: You can also get the raw (non unified values) for */
///* the last data sample as follows. The .getEvent call populates */
///* the raw values used below. */
////Serial.print("X Raw: "); Serial.print(accel.raw.x); Serial.print("  ");
////Serial.print("Y Raw: "); Serial.print(accel.raw.y); Serial.print("  ");
////Serial.print("Z Raw: "); Serial.print(accel.raw.z); Serial.println("");
//
///* Delay before the next sample */
//  delay(100);
}

