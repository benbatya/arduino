/*
fft_adc_serial.pde
guest openmusiclabs.com 7.7.14
example sketch for testing the fft library.
it takes in data on ADC0 (Analog0) and processes them
with the fft. the data is sent out over the serial
port at 115.2kb.
*/

#define LOG_OUT 1 // use the log output function
#define FFT_N 256 // set to 256 point fft

#define LED_PIN 13
#define TIME_DELAY 100

#include <FFT.h> // include the library

void setup() 
{ 
//  pinMode(LED_PIN, OUTPUT);
//  Serial.begin(9600);
    Serial.begin(115200); // use the serial port
//  TIMSK0 = 0; // turn off timer0 for lower jitter
//  ADCSRA = 0xe5; // set adc on, set the adc to free running mode, prescaler=32
//  ADMUX = (0x42); // use adc13
//  DIDR2 = 1<<2; // turn off the digital input for adc2
    
//  ADCSRA = 0;
//  ADCSRB = 0;
//
//  ADMUX = 0;
//
//  ADMUX |= (1 << REFS0); //set reference voltage
//  ADMUX |= (1 << ADLAR); //left align the ADC value- so we can read highest 8 bits from ADCH register only
//
//  ADMUX |= (1 << 1); //
//
//  ADCSRA |= (1 << ADPS2) | (1 << ADPS0); //set ADC clock with 32 prescaler- 16mHz/32=500kHz
//  ADCSRA |= (1 << ADATE); //enabble auto trigger
//  ADCSRA |= (1 << ADEN); //enable ADC
//  ADCSRA |= (1 << ADSC); //start ADC measurements
}

//FPSCounter counter("loop");

int16_t adjust_val(int16_t val);
#define NUM_VALUES 16
static int16_t vals[NUM_VALUES];
static uint16_t val_index = 0;

uint8_t log2x16(uint16_t x);

void loop() 
{ 
//  static uint16_t count = 0;
//  static uint32_t total_time = 0;
    
//  uint32_t start = micros(); // This uses timer0
    
    memset(vals, sizeof(vals), 0);

//  static uint8_t temp_buf[FFT_N/2];

    while (1) // reduces jitter
    {
//      cli();  // UDRE interrupt slows this way down on arduino1.0
        for (int i = 0; i < FFT_N * 2; i += 2) // save FFT_N samples
        {
//          while (!(ADCSRA & 0x10)); // wait for adc to be ready
//          ADCSRA = 0xf5; // restart adc
//          uint8_t m = ADCL; // fetch adc data
//          uint8_t j = ADCH;
//          int16_t k = (j << 8) | m; // form into an int
            int16_t k = analogRead(MIC_PIN);
            k -= 0x0200; // form into a signed int
            k <<= 6; // form into a 16b signed int
            fft_input[i] = k; // put real data into even bins
            fft_input[i + 1] = 0; // set odd bins to 0

//          temp_buf[i/4] = j;
        }
        fft_window(); // window the data for better frequency response
        fft_reorder(); // reorder the data before doing the fft
        fft_run(); // process the data in the fft
        fft_mag_log(); // take the output of the fft
//      sei();
        

//      for (int i=0; i<FFT_N/2; i++)
//      {
//          uint8_t val = fft_lin_out[i]>>8;
//          temp_buf[i] = uint8_t(val);
//      }

        Serial.write(255); // send a start byte
        Serial.write(fft_log_out, FFT_N/2); // send out the data
//      Serial.write(temp_buf, FFT_N/2);
        Serial.flush();

//      Serial.println(val);
//      delay(200);
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

