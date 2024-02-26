#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <MAX3010x.h>
#include <hp_BH1750.h> 
#include "filters.h"

#define TCAADDR 0x70
#define BMP280_ADDRESS 0x76
#define BH1750_ADDRESS 0x23
#define MAX30100_ADDRESS 0x57
Adafruit_BMP280 bmp; // I2C
hp_BH1750 BH1750;       //  create the sensor

// Sensor (adjust to your sensor type)
MAX30105 sensor;
const auto kSamplingRate = sensor.SAMPLING_RATE_400SPS;
const float kSamplingFrequency = 400.0;

// Finger Detection Threshold and Cooldown
const unsigned long kFingerThreshold = 10000;
const unsigned int kFingerCooldownMs = 500;

// Edge Detection Threshold (decrease for MAX30100)
const float kEdgeThreshold = -2000.0;

// Filters
const float kLowPassCutoff = 5.0;
const float kHighPassCutoff = 0.5;

// Averaging
const bool kEnableAveraging = true;
const int kAveragingSamples = 50;
const int kSampleThreshold = 5;

// Filter Instances
HighPassFilter high_pass_filter(kHighPassCutoff, kSamplingFrequency);
LowPassFilter low_pass_filter(kLowPassCutoff, kSamplingFrequency);
Differentiator differentiator(kSamplingFrequency);
MovingAverageFilter<kAveragingSamples> averager;

// Timestamp of the last heartbeat
long last_heartbeat = 0;

// Timestamp for finger detection
long finger_timestamp = 0;
bool finger_detected = false;

// Last diff to detect zero crossing
float last_diff = NAN;
bool crossed = false;
long crossed_time = 0;

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void setup(){
    
  delay(5000);
  Serial.begin(115200);
  Serial.println("Test");

  Wire.begin();
  Serial.println("\nTCAScanner ready!");
  
  for (uint8_t t=0; t<8; t++) {
    tcaselect(t);
    Serial.print("TCA Port #"); Serial.println(t);

    for (uint8_t addr = 0; addr<=127; addr++) {
      if (addr == TCAADDR) continue;

      Wire.beginTransmission(addr);
      if (!Wire.endTransmission()) {
        Serial.print("Found I2C 0x");  Serial.println(addr,HEX);
      }
    }
  }
  Serial.println("\ndone");
  Serial.println("alles effe testen g"); Serial.println("");
  
  /* Initialise the 1st sensor */
  tcaselect(7);
  if(!sensor.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no PPG detected ... Check your wiring!");
    while(1);
  }
  
  /* Initialise the 2nd sensor */
  tcaselect(5);
  bool avail = BH1750.begin(BH1750_ADDRESS);// init the sensor with address pin connetcted to ground
                                              // result (bool) wil be be "false" if no sensor found
  if (!avail) {
    Serial.println("No BH1750 sensor found!");
    while (true) {}; 
  }

  tcaselect(6);
  unsigned status;
  status = bmp.begin(BMP280_ADDRESS);
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,    
                  Adafruit_BMP280::SAMPLING_X2,     
                  Adafruit_BMP280::SAMPLING_X16,    
                  Adafruit_BMP280::FILTER_X16,      
                  Adafruit_BMP280::STANDBY_MS_500); 
  

  Serial.println("Aight lets cook");
}

void loop(){
    tcaselect(7);
    auto sample = sensor.readSample(1000);
    float current_value = sample.red;
    
    // Detect Finger using raw sensor value
    if(sample.red > kFingerThreshold) {
      if(millis() - finger_timestamp > kFingerCooldownMs) {
        finger_detected = true;
      }
    }
    else {
      // Reset values if the finger is removed
      differentiator.reset();
      averager.reset();
      low_pass_filter.reset();
      high_pass_filter.reset();
      
      finger_detected = false;
      finger_timestamp = millis();
    }

    if(finger_detected) {
      current_value = low_pass_filter.process(current_value);
      current_value = high_pass_filter.process(current_value);
      float current_diff = differentiator.process(current_value);

      // Valid values?
      if(!isnan(current_diff) && !isnan(last_diff)) {
        
        // Detect Heartbeat - Zero-Crossing
        if(last_diff > 0 && current_diff < 0) {
          crossed = true;
          crossed_time = millis();
        }
        
        if(current_diff > 0) {
          crossed = false;
        }
    
        // Detect Heartbeat - Falling Edge Threshold
        if(crossed && current_diff < kEdgeThreshold) {
          if(last_heartbeat != 0 && crossed_time - last_heartbeat > 300) {
            // Show Results
            int bpm = 60000/(crossed_time - last_heartbeat);
            if(bpm > 50 && bpm < 250) {
              // Average?
              if(kEnableAveraging) {
                int average_bpm = averager.process(bpm);
    
                // Show if enough samples have been collected
                if(averager.count() > kSampleThreshold) {
                  Serial.print("Heart Rate (avg, bpm): ");
                  Serial.println(average_bpm);
                }
              }
              else {
                Serial.print("Heart Rate (current, bpm): ");
                Serial.println(bpm);  
              }
              tcaselect(6);
              float temp=bmp.readTemperature();
              float pressure=bmp.readPressure();
              float altitude=bmp.readAltitude(1013.25);
                // Print the values
              Serial.print("Temperature: ");
              Serial.print(temp);
              Serial.println(" *C");

              Serial.print("Pressure: ");
              Serial.print(pressure);
              Serial.println(" Pa");

              Serial.print("Altitude: ");
              Serial.print(altitude);
              Serial.println(" meters");

              tcaselect(5);
              BH1750.start();   //starts a measurement
              float lux=BH1750.getLux();  //  waits until a conversion finished
              Serial.print("LUX: ");
              Serial.print(lux);
              Serial.println(" ...");
              Serial.println("");
            }
          }
          crossed = false;
          last_heartbeat = crossed_time;
        }
      }
      last_diff = current_diff;
    }
}