//Test clone
//Laurence demo
#include <Arduino.h>
#include <MAX3010x.h>
#include "filters.h"

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

void setup() {
  Serial.begin(115200);

  delay(5000);
  Serial.println("Test");

  if(sensor.begin() && sensor.setSamplingRate(kSamplingRate)) { 
    Serial.println("Sensor initialized");
  }
  else {
    Serial.println("Sensor not found");  
    while(1);
  }
}

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

void loop() {
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
          }
        }
  
        crossed = false;
        last_heartbeat = crossed_time;
      }
    }

    last_diff = current_diff;
  }
}


/*

//I2CScanner
#include <Arduino.h>
#include <Wire.h>


void setup()
{
  Wire.begin(8,10); //SDA,SCL

  Serial.begin(115200);
  while (!Serial);             // Leonardo: wait for serial monitor
  Serial.println("\nI2C Scanner");
}


void loop()
{
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ){
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    Serial.printf("0X%X, ", address, error);

    if (error == 0){
      Serial.print("\nI2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4){
      Serial.print("\nUnknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }

  if (nDevices == 0)
    Serial.println("\nNo I2C devices found\n");
  else
    Serial.println("done\n");

  delay(5000);           // wait 5 seconds for next scan
}

*/