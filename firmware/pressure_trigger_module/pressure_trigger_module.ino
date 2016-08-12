#include "IntervalTimer.h"
#include "PressurePeakDetect.h"
#include <SPI.h>

IntervalTimer sampletimer;

// written for Teensy 3.1 running at 96 MHz
const uint8_t SAMPLING_PERIOD = 2; // milliseconds
const uint8_t TRIGGER_PULSE_DURATION = 10; // milliseconds, given in the scanner's external triggering timing table
const uint8_t SCANNER_TRIGGER_PIN = 3;
const uint8_t PULSE_DURATION = TRIGGER_PULSE_DURATION / SAMPLING_PERIOD;
volatile uint8_t pulseDurationCount;
volatile bool triggerPulseHigh = false;

const uint8_t ANALOG_INPUT_CONTROL_BYTE = 0x8C;
const uint8_t MODE_CONTROL_BYTE = 0x88;
const uint8_t CONVERSION_START_BYTE = 0x80;
const uint8_t SLAVE_SELECT_PIN = 10;

volatile uint32_t result, msb, lsb;

lowPassFilter filt;
slopeSumFilter ssf;
peakDetect pd;

void setup() {
    Serial.begin(115200); // fastest stable BAUD rate (Hz)
    pinMode(SCANNER_TRIGGER_PIN, OUTPUT);
    setup_ADC();
    // hardware clock interrupt service routine calls the sample routine every x microseconds
    sampletimer.begin(sample, SAMPLING_PERIOD * 1000);
}

void loop() {

}

void sample() {
    // signal pathway
    // blood pressure transducer --> Arduino ADC --> low pass filter --> slopesum function --> peak detector
    volatile uint32_t sampleVal = ADC_conversion();
    volatile uint32_t lpfVal = filt.step(sampleVal);
    volatile uint32_t ssfVal = ssf.step(lpfVal);
    volatile bool sampleIsPeak = pd.isPeak(ssfVal);

    if(sampleIsPeak) {
        // when a peak is detected, sent a TTL pulse to the scanner
        digitalWrite(SCANNER_TRIGGER_PIN, HIGH);
        triggerPulseHigh = true;
        pulseDurationCount = 0;
    }

    if (triggerPulseHigh) {
        if (pulseDurationCount >= PULSE_DURATION) {
            // keep the TTL pulse at logic high (3.3V) until pulse duration exceeded 
            digitalWrite(SCANNER_TRIGGER_PIN, LOW);
            triggerPulseHigh = false;
        } else {
            pulseDurationCount++;
        }
    }
    Serial.printf("%d %d\n", sampleVal, triggerPulseHigh);
    Serial.send_now();
}

uint32_t ADC_conversion() {
  // gain control of the SPI port
  // and configure settings
  // take the SS pin low to select the chip:
  digitalWrite(SLAVE_SELECT_PIN, LOW);
  //  send in the address and value via SPI
  
  SPI.transfer(CONVERSION_START_BYTE);
  SPI.transfer(0); // throwaway byte
  msb = SPI.transfer(0);
  lsb = SPI.transfer(0); // DIN on the ADC is ignored, so simply send 00000000
  // take the SS pin high to de-select the chip:
  digitalWrite(SLAVE_SELECT_PIN,HIGH);
  // release control of the SPI port
  result = msb << 8 | lsb;
  return result;
}

void setup_ADC() {
  // gain control of the SPI port
  // and configure settings
  // take the SS pin low to select the chip:
  SPI.begin();
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  digitalWrite(SLAVE_SELECT_PIN, LOW);
  SPI.transfer(ANALOG_INPUT_CONTROL_BYTE);
  digitalWrite(SLAVE_SELECT_PIN, HIGH);

  // take the SS pin low to select the chip:
  digitalWrite(SLAVE_SELECT_PIN, LOW);
  SPI.transfer(MODE_CONTROL_BYTE);
  digitalWrite(SLAVE_SELECT_PIN, HIGH);
}
