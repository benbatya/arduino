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

#define DISPLAY_OUTPUT 1
//#define DISPLAY_AMP 1

#ifndef DISPLAY_OUTPUT
uint32_t prev_time;
static uint8_t max_vals[FFT_N/2];
#endif

void setup() 
{ 
#if DISPLAY_OUTPUT
    Serial.begin(115200); // use the serial port
#else
    Serial.begin(9600);
#endif

    analogReference(DEFAULT);

    // Change the prescaler to get a better sampling rate
    ADCSRA &= ~0x07;
    ADCSRA |= _BV(ADPS2) | _BV(ADPS0); // prescaler = 32

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
//
//#if DISPLAY_OUTPUT
////#define REMOVE_NOISE 1
//#endif
//
//#if REMOVE_NOISE
//
//static const uint8_t PROGMEM noise[128] = {
//    8, 5, 4, 4, 3, 4, 5, 4, 4, 9, 10, 11, 9, 7, 4, 3
//    , 3, 3, 3, 3, 3, 5, 10, 12, 8, 6, 4, 3, 2, 2, 2, 2
//    , 2, 4, 7, 5, 6, 6, 4, 2, 3, 3, 2, 2, 4, 7, 7, 4
//    , 3, 2, 1, 2, 1, 2, 2, 2, 6, 6, 4, 4, 3, 2, 2, 2
//    , 2, 2, 1, 4, 6, 5, 2, 2, 2, 2, 2, 2, 2, 1, 3, 6
//    , 6, 3, 4, 3, 3, 3, 2, 2, 2, 2, 5, 7, 5, 4, 3, 3
//    , 3, 3, 3, 3, 3, 7, 9, 8, 4, 5, 4, 5, 5, 6, 4, 4
//    , 5, 7, 8, 5, 6, 6, 4, 3, 3, 4, 3, 3, 2, 3, 3, 4
//};
//
//
//#else
//
//static const uint8_t PROGMEM  noise[128] = {
//    // This is low-level noise that's subtracted from each FFT output column:
//    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
//    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
//    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
//    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
//    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
//    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
//    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
//    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
//};
//#endif
//
//#define INPUT_FLOOR 19 //Lower range of analogRead input
//#define INPUT_CEILING 600 //Max range of analogRead input, the lower the value the more sensitive (1023 = max)
//float fscale(float originalMin, float originalMax, float newBegin, float
//           newEnd, float inputValue, float curve);
//
//uint8_t log2x16(uint16_t x);

void loop() 
{  
//  uint32_t start_time = micros();

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

//  total += micros() - start_time;
//  count++;

#if DISPLAY_AMP

    int16_t max, min;
    max = min = fft_input[0];
    for (int i=1; i<FFT_N; i++)
    {
        int16_t val = fft_input[i*2];
        if (val < min) { min = val; }
        else if (val > max) { max = val; }
    }
    uint16_t diff = max - min;
    diff >>= 8;
//  uint16_t diff = max<0 ? -max: max;
//  uint16_t w = fscale(INPUT_FLOOR, INPUT_CEILING, 0, 255, diff, 2);
//  uint8_t val = log2x16(diff);
    uint8_t val = diff;

    for (int i=FFT_N/2-1; i>0; i--) 
    {
        FFT_OUTPUT[i] = FFT_OUTPUT[i-1];
    }
    FFT_OUTPUT[0] = val;

#else
    fft_window(); // window the data for better frequency response
    fft_reorder(); // reorder the data before doing the fft
    fft_run(); // process the data in the fft
    FFT_FUNC(); // take the output of the fft

    // NOTE: I only could remove the noise filters after turning down the gain to zero.
//  for (int i=0; i<FFT_N/2; i++)
//  {
//      // Do a temporal filter operation
//      uint8_t L = pgm_read_byte(&noise[i]);
//      FFT_OUTPUT[i] = (FFT_OUTPUT[i] <= L) ? 0 : FFT_OUTPUT[i];
//  }
#endif  

#if DISPLAY_OUTPUT

    Serial.write(255); // send a start byte
    Serial.write(FFT_OUTPUT, FFT_N/2); // send out the data
//  Serial.write(output, FFT_N/2); // send out the data

#else

//  if (count > 50)
//  {
//      float avg = float(total) / count;
//      total = 0;
//      count = 0;
//      Serial.print("average time = "); Serial.print(avg);
//      float hz = avg / FFT_N;
//      hz = 1000000.0f/hz;
//      Serial.print(", Hz = "); Serial.println(hz);
//  }

    for (int i=0; i<FFT_N/2; i++)
    {
        if (max_vals[i] < FFT_OUTPUT[i])
        {
            max_vals[i] = FFT_OUTPUT[i];
        }
    }

    if (millis() - prev_time > 1000)
    {
        Serial.print("static const uint8_t PROGMEM noise["); Serial.print(FFT_N/2); Serial.print("] = {");
        const char* sep = "";
        for (int i=0; i<FFT_N/2; i++)
        {
            if (i%32 == 0)
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

//  Serial.flush();
}

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
//
//    }
//    else     // invert the ranges
//    {
//        rangedValue =  newBegin - (pow(normalizedCurVal, curve) * NewRange);
//    }
//
//    return rangedValue;
//}
//
//// From http://www.avrfreaks.net/comment/567704#comment-567704
//// Table 16*(Log2(1) thru Log2(2)), 16 values
//static const uint8_t Log2Table[16]={ 0,2,3,4,5,6,7,8,9,10,11,12,13,14,14,15 };
//
//// Log2(x)*16 , so log2x16(65535) = 255
//uint8_t log2x16(uint16_t x){
//    int8_t v;
//    if(x<2) return 0;
//    v=240; // 16*log2(2^15)=240
//    // shift until most significant bit is set,
//    // each bit-shift results in log += 16
//    while ((x&0x8000)==0) {
//        x=x<<1 ; v -= 16;
//    }
//    // x has now form 1iii ixxx xxxx xxxx
//    // get the next 4 bits =iiii and address table with it
//    uint8_t i=(x>>11) & 0xf ;
//    v += Log2Table[i] ;
//    return v ;
//}

