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
}

//FPSCounter counter("loop");

int16_t adjust_val(int16_t val);
#define NUM_VALUES 16
static int16_t vals[NUM_VALUES];
static uint16_t val_index = 0;

uint8_t log2x16(uint16_t x);

static const uint8_t PROGMEM
  // This is low-level noise that's subtracted from each FFT output column:
  noise[128]={  100,90,80,80,80,80,80,80,80,80,80,70,70,70,70,70,
                70,70,70,70,70,70,70,70,70,70,80,70,70,70,70,70,
                96,106,90,70,70,70,70,70,70,70,70,70,70,70,70,70,
                70,70,70,70,70,70,70,70,70,70,80,86,80,70,70,70,
                70,105,110,85,70,70,70,70,70,70,70,70,70,70,70,70,
                70,70,70,70,70,70,70,70,70,70,70,90,90,80,70,70,
                70,70,105,105,80,70,70,70,70,70,70,70,70,70,70,70,
                70,70,70,70,70,70,70,70,70,70,70,70,95,100,70,70};

void loop() 
{ 
//  static uint16_t count = 0;
//  static uint32_t total_time = 0;
    
//  uint32_t start = micros(); // This uses timer0
    
    memset(vals, sizeof(vals), 0);

    while (1) // reduces jitter
    {
        cli();  // UDRE interrupt slows this way down on arduino1.0
        for (int i = 0; i < FFT_N * 2; i += 2) // save FFT_N samples
        {
//          while (!(ADCSRA & 0x10)); // wait for adc to be ready
//          ADCSRA = 0xf5; // restart adc
//          uint8_t m = ADCL; // fetch adc data
//          uint8_t j = ADCH;
//          int16_t k = (j << 8) | m; // form into an int

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

//      static bins[3] = { 0 };
//
//      for (int i=0; i<3; i++)
//      {
//          bins[i] = 0;
//      }
//
//      int8_t current_bin = -1;
//      for (int i=0; i<FFT_N/2; i++)
//      {
//          uint8_t val = output[i];
//          for (int j=0; j<8; j++)
//          {
//              if (val >= (1<<j))
//              {
//                  bins[current_bin] += 1;
//              }
//              else
//              {
//                  break;
//              }
//          }
//
//          if (i%86 == 85)
//          {
//              current_bin++;
//          }
//      }

        Serial.write(255); // send a start byte
        Serial.write(output, FFT_N/2); // send out the data
        Serial.flush();

//      Serial.println(val);
//      delay(200);
    }
}



