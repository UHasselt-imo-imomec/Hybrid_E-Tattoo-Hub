#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <MAX3010x.h>
#include <hp_BH1750.h> 
#include "filters.h"
#include <ESP32AnalogRead.h>

// Define I2C Addresses
#define TCAADDR 0x70
#define BMP280_ADDRESS 0x76
#define BH1750_ADDRESS 0x23
#define MAX30100_ADDRESS 0x57
Adafruit_BMP280 bmp; // I2C
hp_BH1750 BH1750;       //  create the sensor


//Timings for sensor readout:
#define PPG_SENSOR_READOUT_TIME_MS 10
#define TEMP_HUM_SENSOR_READOUT_TIME_MS 1000
#define ANALOG_TEMP_SENSOR_READOUT_TIME_MS 1000
#define LIGHT_SENSOR_READOUT_TIME_MS 5000
unsigned long last_PPG_sensor_readout = 0;
unsigned long last_TEMP_HUM_sensor_readout = 0;
unsigned long last_LIGHT_sensor_readout = 0;
unsigned long last_ANALOG_TEMP_sensor_readout = 0;
bool readPPG = false;
bool readTEMP_HUM = false;
bool readLIGHT = false;
bool readAnalog_TEMP = false;


// analoge sensor
// Define the number of samples to keep track of. The higher the number, the
// more the readings will be smoothed, but the slower the output will respond to
// the input.
const int numReadings = 10;
int analogPin = A3;
int readIndex = 0;            // the index of the current reading
float readings[numReadings];  // the readings from the analog input
float total = 0;              // the running total
float average_Vout = 0;       // the average voltage
float Vin = 3.3;
float R1 = 0;
float R2 = 220.5;
ESP32AnalogRead adc;

// Predefined sensor channels (MUX channel#)
enum sensorchannels {PPG_SENSOR = 7, TEMP_HUM_SENSOR = 6, LIGHT_SENSOR = 5};

// PPG_sensor (adjust to your sensor type)
MAX30105 PPG_sensor;    //Controleren, ik ben niet zeker of dit de juiste sensor is!
const auto kSamplingRate = PPG_sensor.SAMPLING_RATE_400SPS;
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
    
  delay(5000);    //Delay to let Serial Monitor catch up (Because of CDCBoot)
  adc.attach(analogPin);
  Serial.begin(115200);
  Serial.println("Initializing");

  Wire.begin();
  Serial.println("\nTCAScanner ready!");  //Scan all TCA ports for I2C devices, and report back immediately upon finding one.

  for (uint8_t t=0; t<8; t++) {
    tcaselect(t);
    Serial.print("TCA Port #"); Serial.println(t);

    for (uint8_t addr = 0; addr<=127; addr++) {
      if (addr == TCAADDR) continue;

      Wire.beginTransmission(addr);
      if (!Wire.endTransmission()) {
        Serial.printf("Found I2C 0x%X on TCA Port #%d.\n",addr, t);
      }
    }
  }
  Serial.println("\ndone");
  Serial.println("alles effe testen"); Serial.println("");
  
  /* Initialise the 1st PPG_sensor */ //--> Dit kan dynamisch gemaakt worden, maar ik zou hier niet te veel tijd aan besteden! 
  tcaselect(PPG_SENSOR);
  if(!PPG_sensor.begin())  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no PPG detected ... Check your wiring!");
    while(1) delay(10);
  }

  
  /* Initialise the 2nd sensor (Light sensor)*/
  tcaselect(LIGHT_SENSOR);
  bool avail = BH1750.begin(BH1750_ADDRESS);// init the sensor with address pin connetcted to ground
                                              // result (bool) wil be be "false" if no sensor found
  if (!avail) {
    Serial.println("No BH1750 sensor found!");
    while (true) {}; 
  }

  /* Initialise the 3rd sensor (Temp/Hum sensor)*/
  tcaselect(TEMP_HUM_SENSOR);
  unsigned status;
  status = bmp.begin(BMP280_ADDRESS);
  if (!status) {  //List possible errors
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
  
  for (int thisReading = 0; thisReading < numReadings; thisReading++) { // initialize all the readings of analog sensor to 0:
    readings[thisReading] = 0;
  }

  Serial.println("Aight lets cook");
}

void loop(){
  unsigned long current_time = millis();  //Check if sensors have to be read
  if (current_time - last_PPG_sensor_readout > PPG_SENSOR_READOUT_TIME_MS){
    readPPG = true;
  }
  if (current_time - last_TEMP_HUM_sensor_readout > TEMP_HUM_SENSOR_READOUT_TIME_MS){
    readTEMP_HUM = true;
  }
  if (current_time - last_LIGHT_sensor_readout > LIGHT_SENSOR_READOUT_TIME_MS){
    readLIGHT = true;
  }
  if (current_time - last_ANALOG_TEMP_sensor_readout > ANALOG_TEMP_SENSOR_READOUT_TIME_MS){
    readAnalog_TEMP = true;
  }

  uint32_t ir_value;
  uint32_t red_value;
  float temp;
  float altitude;
  float pressure;
  float lux;

  if(readPPG){
    readPPG = false;
    tcaselect(PPG_SENSOR);
    auto sample = PPG_sensor.readSample(1000); // De 1000 is de timeout in ms, niet het aantal samples!
    if (sample.valid){  //Check if the sample is valid, then proceed
      ir_value = sample.ir;
      red_value = sample.red;
      //Serial.printf("IR: %u, Red: %u\n", ir_value, red_value);
    }
    else{
      //Serial.println("Sample not valid, probably the timeout is too short!");
    }
  }
    
  if(readTEMP_HUM){  
    readTEMP_HUM = false;
    tcaselect(TEMP_HUM_SENSOR);
    temp=bmp.readTemperature();
    pressure=bmp.readPressure();
    altitude=bmp.readAltitude(1013.25);

    //Serial.printf("Temperature: %f, Pressure: %f, Altitude: %f\n", temp, pressure, altitude);
  }

  if(readLIGHT){
    readLIGHT = false;
    tcaselect(LIGHT_SENSOR);
    BH1750.start();
    lux=BH1750.getLux();
    //Serial.printf("Light: %f\n", lux);
  }

  if(readAnalog_TEMP){
    readAnalog_TEMP = false;
        // subtract the last reading:
    total = total - readings[readIndex];
    // read from the sensor:
    readings[readIndex] = adc.readVoltage();
    // add the reading to the total:
    total = total + readings[readIndex];
    // advance to the next position in the array:
    readIndex = readIndex + 1;

    // if we're at the end of the array...
    if (readIndex >= numReadings) {
      // ...wrap around to the beginning:
      readIndex = 0;
    }

    // calculate the average:
    average_Vout = total / numReadings;
    R1 = R2 * ((Vin/average_Vout) - 1);
    //Serial.printf("Vout: %f, R1: %f\n", average_Vout, R1);
  }
  Serial.print("/*" + String(millis()) + "," + ir_value + "," + red_value + "," + temp + "," + pressure + "," + altitude + "," + lux + "," + R1 + "*/");
  Serial.println();  // <- Print an end of line, fixed the issue for me.
}