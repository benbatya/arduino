/****************************************
 * Example Sound Level Sketch for the 
 * Adafruit Microphone Amplifier
 ****************************************/

const unsigned long SAMPLE_DELAY = 100;

#ifndef MIC_PIN
#define  MIC_PIN 0 // This is the analog audio pin on the tie, it may need to be configurable
#endif

void setup() 
{
  Serial.begin(9600);
}

double amplitude();

void loop() 
{
  unsigned long start = millis();

  double volts = amplitude();
  
  unsigned long total_millis = millis() - start;
  
  Serial.println(volts);
  
  unsigned long delay_millis = SAMPLE_DELAY - total_millis;
  delay(delay_millis);
}


double amplitude()
{
  static const int sampleWindow = 50; // Sample window width in mS (50 mS = 20Hz)
  unsigned long startMillis= millis();  // Start of sample window
  unsigned int peakToPeak = 0;   // peak-to-peak level

  unsigned int signalMax = 0;
  unsigned int signalMin = 1024;
  
  // collect data for 50 mS
  while (millis() - startMillis < sampleWindow)
  {
    unsigned int sample = analogRead(MIC_PIN);
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
  peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
  double volts = (peakToPeak * 3.3) / 1024;  // convert to volts 
  
  return volts;
}

