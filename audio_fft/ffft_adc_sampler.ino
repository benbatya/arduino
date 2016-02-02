// Use FrequencyTimer2 to avoid problems with ADC_vect using TIMER0
#include <FrequencyTimer2.h>

// IMPORTANT: FFT_N should be #defined as 128 in ffft.h.
//#include <ffft.h>

//#define FFT_N 256 // set to 256 point fft
#include <FFT.h>

//#include <math.h>
//#include <Wire.h>

#define AUDIO_PIN 0

volatile uint16_t samplePos = 0;     // Buffer position counter
int16_t  capture[FFT_N];    // Audio capture buffer
//complex_t     bfly_buff[FFT_N];  // FFT "butterfly" buffer
//uint16_t      spectrum[FFT_N/2]; // Spectrum output buffer

byte
  peak[8],      // Peak level of each column; used for falling dots
  dotCount = 0, // Frame counter for delaying dot-falling speed
  colCount = 0; // Frame counter for storing past column data
int
  col[8][10],   // Column levels for the prior 10 frames
  minLvlAvg[8], // For dynamic adjustment of low & high ends of graph,
  maxLvlAvg[8], // pseudo rolling averages for the prior few frames.
  colDiv[8];    // Used when filtering FFT output to 8 columns

/*
These tables were arrived at through testing, modeling and trial and error,
exposing the unit to assorted music and sounds.  But there's no One Perfect
EQ Setting to Rule Them All, and the graph may respond better to some
inputs than others.  The software works at making the graph interesting,
but some columns will always be less lively than others, especially
comparing live speech against ambient music of varying genres.
*/
static const uint8_t PROGMEM
  // This is low-level noise that's subtracted from each FFT output column:
  noise[64]={ 8,6,6,5,3,4,4,4,3,4,4,3,2,3,3,4,
              2,1,2,1,3,2,3,2,1,2,3,1,2,3,4,4,
              3,2,2,2,2,2,2,1,3,2,2,2,2,2,2,2,
              2,2,2,2,2,2,2,2,2,2,2,2,2,3,3,4 },
  // These are scaling quotients for each FFT output column, sort of a
  // graphic EQ in reverse.  Most music is pretty heavy at the bass end.
  eq[64]={
    255, 175,218,225,220,198,147, 99, 68, 47, 33, 22, 14,  8,  4,  2,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
  // When filtering down to 8 columns, these tables contain indexes
  // and weightings of the FFT spectrum output values to use.  Not all
  // buckets are used -- the bottom-most and several at the top are
  // either noisy or out of range or generally not good for a graph.
  col0data[] = {  2,  1,  // # of spectrum bins to merge, index of first
    111,   8 },           // Weights for each bin
  col1data[] = {  4,  1,  // 4 bins, starting at index 1
     19, 186,  38,   2 }, // Weights for 4 bins.  Got it now?
  col2data[] = {  5,  2,
     11, 156, 118,  16,   1 },
  col3data[] = {  8,  3,
      5,  55, 165, 164,  71,  18,   4,   1 },
  col4data[] = { 11,  5,
      3,  24,  89, 169, 178, 118,  54,  20,   6,   2,   1 },
  col5data[] = { 17,  7,
      2,   9,  29,  70, 125, 172, 185, 162, 118, 74,
     41,  21,  10,   5,   2,   1,   1 },
  col6data[] = { 25, 11,
      1,   4,  11,  25,  49,  83, 121, 156, 180, 185,
    174, 149, 118,  87,  60,  40,  25,  16,  10,   6,
      4,   2,   1,   1,   1 },
  col7data[] = { 37, 16,
      1,   2,   5,  10,  18,  30,  46,  67,  92, 118,
    143, 164, 179, 185, 184, 174, 158, 139, 118,  97,
     77,  60,  45,  34,  25,  18,  13,   9,   7,   5,
      3,   2,   2,   1,   1,   1,   1 },
  // And then this points to the start of the data for each of the columns:
  * const colData[]  = {
    col0data, col1data, col2data, col3data,
    col4data, col5data, col6data, col7data };

const static int led_pin = 13;
//const static int AUDIO_PIN = 1<<PC0; // use raw ATMEL c port manipulation

int32_t time_total = 0;
uint32_t start = 0;

// variables shared between interrupt context and main program
// context must be declared "volatile".
volatile uint32_t sample_count = 0;

void update_sample(void);

void setup() {
    pinMode(led_pin, OUTPUT);
    Serial.begin(9600);

//  DDRC &= ~AUDIO_PIN;

//uint8_t i, j, nBins, binNum, *data;

//   memset(peak, 0, sizeof(peak));
//   memset(col , 0, sizeof(col));

//for(i=0; i<8; i++) {
//  minLvlAvg[i] = 0;
//  maxLvlAvg[i] = 512;
//  data         = (uint8_t *)pgm_read_word(&colData[i]);
//  nBins        = pgm_read_byte(&data[0]) + 2;
//  binNum       = pgm_read_byte(&data[1]);
//  for(colDiv[i]=0, j=2; j<nBins; j++)
//    colDiv[i] += pgm_read_byte(&data[j]);
//}
//
//matrix.begin(0x70);

    start = micros();

    pinMode(FREQUENCYTIMER2_PIN, OUTPUT);
    // This gives us a simple way to sample the data
    FrequencyTimer2::setPeriod(100);
    FrequencyTimer2::setOnOverflow(update_sample);
    FrequencyTimer2::enable();

//  Serial.println("Running!");
}

void loop() {
//  uint8_t  i, *data, nBins, binNum, weighting, c;
//uint16_t minLvl, maxLvl;
//int      level, y, sum;

    while(samplePos < FFT_N) ;

//  uint32_t start = micros();

    // Wait for the FFT buffer to get filled
    
//  FrequencyTimer2::disable();

//  Serial.println("filled buffer");

    int val = capture[0];
//  fft_input[0] = val;

//  for (int i=0; i<FFT_N; i++)
//  {
//      fft_input[i*2+0] = capture[i];
//      fft_input[i*2+1] = 0;
//  }

//  fft_input(capture, bfly_buff);   // Samples -> complex #s
    samplePos = 0;                   // Reset sample counter

//  FrequencyTimer2::enable();

//  fft_execute(bfly_buff);          // Process complex data
//  fft_output(bfly_buff, spectrum); // Complex -> spectrum
//
//  // Remove noise and apply EQ levels
//  for(uint8_t x=0; x<FFT_N/2; x++) {
//      uint8_t L = pgm_read_byte(&noise[x]);
//      spectrum[x] = (spectrum[x] <= L) ? 0 :
//          (((spectrum[x] - L) * (256L - pgm_read_byte(&eq[x]))) >> 8);
//  }

    sample_count++;

//  int32_t diff = micros() - start;
//  time_total += diff;
//  sample_count++;

    if (sample_count >= 10) 
    {
        uint32_t curr_time = micros();

        int32_t total_time = curr_time - start;
        start = curr_time;
        Serial.println(float(total_time) / sample_count);

        Serial.println(val);

//      start = millis();
//      digitalWrite(led_pin, HIGH);
//      delay(200);
//      digitalWrite(led_pin, LOW);

//      Serial.begin(9600);
//      Serial.print("Average time to collect samples = ");
//      Serial.println(time_total);
//      Serial.println(sample_count);

        sample_count = 0;
//      time_total = 0;
    }

//// Fill background w/colors, then idle parts of columns will erase
//matrix.fillRect(0, 0, 8, 3, LED_RED);    // Upper section
//matrix.fillRect(0, 3, 8, 2, LED_YELLOW); // Mid
//matrix.fillRect(0, 5, 8, 3, LED_GREEN);  // Lower section
//
//// Downsample spectrum output to 8 columns:
//for(x=0; x<8; x++) {
//  data   = (uint8_t *)pgm_read_word(&colData[x]);
//  nBins  = pgm_read_byte(&data[0]) + 2;
//  binNum = pgm_read_byte(&data[1]);
//  for(sum=0, i=2; i<nBins; i++)
//    sum += spectrum[binNum++] * pgm_read_byte(&data[i]); // Weighted
//  col[x][colCount] = sum / colDiv[x];                    // Average
//  minLvl = maxLvl = col[x][0];
//  for(i=1; i<10; i++) { // Get range of prior 10 frames
//    if(col[x][i] < minLvl)      minLvl = col[x][i];
//    else if(col[x][i] > maxLvl) maxLvl = col[x][i];
//  }
//  // minLvl and maxLvl indicate the extents of the FFT output, used
//  // for vertically scaling the output graph (so it looks interesting
//  // regardless of volume level).  If they're too close together though
//  // (e.g. at very low volume levels) the graph becomes super coarse
//  // and 'jumpy'...so keep some minimum distance between them (this
//  // also lets the graph go to zero when no sound is playing):
//  if((maxLvl - minLvl) < 8) maxLvl = minLvl + 8;
//  minLvlAvg[x] = (minLvlAvg[x] * 7 + minLvl) >> 3; // Dampen min/max levels
//  maxLvlAvg[x] = (maxLvlAvg[x] * 7 + maxLvl) >> 3; // (fake rolling average)
//
//  // Second fixed-point scale based on dynamic min/max levels:
//  level = 10L * (col[x][colCount] - minLvlAvg[x]) /
//    (long)(maxLvlAvg[x] - minLvlAvg[x]);
//
//  // Clip output and convert to byte:
//  if(level < 0L)      c = 0;
//  else if(level > 10) c = 10; // Allow dot to go a couple pixels off top
//  else                c = (uint8_t)level;
//
//  if(c > peak[x]) peak[x] = c; // Keep dot on top

//  if(peak[x] <= 0) { // Empty column?
//    matrix.drawLine(x, 0, x, 7, LED_OFF);
//    continue;
//  } else if(c < 8) { // Partial column?
//    matrix.drawLine(x, 0, x, 7 - c, LED_OFF);
//  }
//
//  // The 'peak' dot color varies, but doesn't necessarily match
//  // the three screen regions...yellow has a little extra influence.
//  y = 8 - peak[x];
//  if(y < 2)      matrix.drawPixel(x, y, LED_RED);
//  else if(y < 6) matrix.drawPixel(x, y, LED_YELLOW);
//  else           matrix.drawPixel(x, y, LED_GREEN);
//}
//
//matrix.writeDisplay();
//
//// Every third frame, make the peak pixels drop by 1:
//if(++dotCount >= 3) {
//  dotCount = 0;
//  for(x=0; x<8; x++) {
//    if(peak[x] > 0) peak[x]--;
//  }
//}
//
//  if(++colCount >= 10) colCount = 0;
}
//
//ISR(ADC_vect) { // Audio-sampling interrupt
//  static const int16_t noiseThreshold = 4;
//  int16_t              sample         = ADC; // 0-1023
//
//  capture[samplePos] =
//    ((sample > (512-noiseThreshold)) &&
//     (sample < (512+noiseThreshold))) ? 0 :
//    sample - 512; // Sign-convert for FFT; -512 to +511
//
//  if(++samplePos >= FFT_N) ADCSRA &= ~_BV(ADIE); // Buffer full, interrupt off
//}

void update_sample(void)
{
    if (samplePos >= FFT_N) 
    {
        return;
    }
    
    static const int16_t noiseThreshold = 4;
    int16_t              sample         = analogRead(AUDIO_PIN);

    capture[samplePos] =
      ((sample > (512-noiseThreshold)) &&
       (sample < (512+noiseThreshold))) ? 0 :
      sample - 512; // Sign-convert for FFT; -512 to +511
    samplePos++;   
}
