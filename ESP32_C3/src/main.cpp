#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <MAX3010x.h>
#include <hp_BH1750.h> 
#include "filters.h"
#include <ESP32AnalogRead.h>
#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>
#include "secrets.h"
#include <Adafruit_NeoPixel.h>

#if defined(ESP32)
#include <WiFiMulti.h>
WiFiMulti wifiMulti;
#define DEVICE "ESP32"
#elif defined(ESP8266)
#include <ESP8266WiFiMulti.h>
ESP8266WiFiMulti wifiMulti; //Demo
#define DEVICE "ESP8266"
#endif

// WiFi AP SSID
#define WIFI_SSID SSID
// WiFi password
#define WIFI_PASSWORD Password
#define INFLUXDB_URL url
#define INFLUXDB_TOKEN TOKEN
#define INFLUXDB_ORG ORG
#define INFLUXDB_BUCKET BUCKET
// Time zone info
#define TZ_INFO "UTC-1"

// Declare InfluxDB client instance with preconfigured InfluxCloud certificate
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN, InfluxDbCloud2CACert);

// Declare Data point
Point X("Sensoren_data");

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
#define ANALOG_TEMP_SENSOR_1_READOUT_TIME_MS 100
#define ANALOG_TEMP_SENSOR_2_READOUT_TIME_MS 100
#define ANALOG_TEMP_SENSOR_3_READOUT_TIME_MS 100
#define ANALOG_TEMP_SENSOR_4_READOUT_TIME_MS 100
#define ANALOG_TEMP_SENSOR_5_READOUT_TIME_MS 100
#define LIGHT_SENSOR_READOUT_TIME_MS 5000
#define LED_DELAY 1000
unsigned long last_PPG_sensor_readout = 0;
unsigned long last_TEMP_HUM_sensor_readout = 0;
unsigned long last_LIGHT_sensor_readout = 0;
unsigned long last_ANALOG_TEMP_sensor_1_readout = 0;
unsigned long last_ANALOG_TEMP_sensor_2_readout = 0;
unsigned long last_ANALOG_TEMP_sensor_3_readout = 0;
unsigned long last_ANALOG_TEMP_sensor_4_readout = 0;
unsigned long last_ANALOG_TEMP_sensor_5_readout = 0;
unsigned long last_blink = 0;
bool readPPG = false;
bool readTEMP_HUM = false;
bool readLIGHT = false;
bool readAnalog_TEMP_1 = false;
bool readAnalog_TEMP_2 = false;
bool readAnalog_TEMP_3 = false;
bool readAnalog_TEMP_4 = false;
bool readAnalog_TEMP_5 = false;
bool readLed = false;


// analoge sensor
// Define the number of samples to keep track of. The higher the number, the
// more the readings will be smoothed, but the slower the output will respond to
// the input.
const int numReadings_sensor_1 = 100;
int analogPin_sensor_1 = 1;
int readIndex_sensor_1 = 0;            // the index of the current reading
float readings_sensor_1[numReadings_sensor_1];  // the readings from the analog input
float total_sensor_1 = 0;              // the running total
float average_Vout_sensor_1 = 0;       // the average voltage
float Vin_sensor_1 = 3.3;
float R_temp_sensor_1 = 0;
float R2 = 220;
ESP32AnalogRead adc1;

const int numReadings_sensor_2 = 100;
int analogPin_sensor_2 = 2;
int readIndex_sensor_2 = 0;            // the index of the current reading
float readings_sensor_2[numReadings_sensor_2];  // the readings from the analog input
float total_sensor_2 = 0;              // the running total
float average_Vout_sensor_2 = 0;       // the average voltage
float Vin_sensor_2 = 3.3;
float R_temp_sensor_2 = 0;
ESP32AnalogRead adc2;

const int numReadings_sensor_3 = 100;
int analogPin_sensor_3 = 3;
int readIndex_sensor_3 = 0;            // the index of the current reading
float readings_sensor_3[numReadings_sensor_3];  // the readings from the analog input
float total_sensor_3 = 0;              // the running total
float average_Vout_sensor_3 = 0;       // the average voltage
float Vin_sensor_3 = 3.3;
float R_temp_sensor_3 = 0;
ESP32AnalogRead adc3;

const int numReadings_sensor_4 = 100;
int analogPin_sensor_4 = 0;
int readIndex_sensor_4 = 0;            // the index of the current reading
float readings_sensor_4[numReadings_sensor_4];  // the readings from the analog input
float total_sensor_4 = 0;              // the running total
float average_Vout_sensor_4 = 0;       // the average voltage
float Vin_sensor_4 = 3.3;
float R_temp_sensor_4 = 0;
ESP32AnalogRead adc4;

const int numReadings_sensor_5 = 100;
int analogPin_sensor_5 = 4;
int readIndex_sensor_5 = 0;            // the index of the current reading
float readings_sensor_5[numReadings_sensor_5];  // the readings from the analog input
float total_sensor_5 = 0;              // the running total
float average_Vout_sensor_5 = 0;       // the average voltage
float Vin_sensor_5 = 3.3;
float R_temp_sensor_5 = 0;
ESP32AnalogRead adc5;
// Predefined sensor channels (MUX channel#)
enum sensorchannels {PPG_SENSOR = 0, TEMP_HUM_SENSOR = 2, LIGHT_SENSOR = 3};

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

Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, 7, NEO_GRB + NEO_KHZ800);

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void setup(){
  
  strip.begin();
  strip.setBrightness(50);
  strip.show(); // Initialize all pixels to 'off'

  delay(5000);    //Delay to let Serial Monitor catch up (Because of CDCBoot)

  Serial.begin(115200);
  Serial.println("Initializing");
  
  Wire.begin();
  Serial.println("\nTCAScanner ready!");  //Scan all TCA ports for I2C devices, and report back immediately upon finding one.

  
  Serial.println("\ndone");
  Serial.println("alles effe testen"); Serial.println("");
  

  /* Initialise the 2nd sensor (Light sensor)*/
  bool avail = BH1750.begin(BH1750_ADDRESS);// init the sensor with address pin connetcted to ground
                                              // result (bool) wil be be "false" if no sensor found
  if (!avail) {
    Serial.println("No BH1750 sensor found!");
    while (true) {}; 
  }
}

void loop(){

    BH1750.start();
    float lux=BH1750.getLux();
    Serial.printf("Light: %f\n", lux);

}
