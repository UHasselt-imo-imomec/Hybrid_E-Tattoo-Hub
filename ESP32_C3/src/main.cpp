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
  delay(5000);    //Delay to let Serial Monitor catch up (Because of CDC_Boot)
  Serial.begin(115200);
  Serial.println("Initializing");

  Wire.begin(8,10,100000);  //SDA, SCL, Frequency
  Serial.println("\nTCAScanner ready!");  //Scan all TCA ports for I2C devices, and report back immediately upon finding one.
  
  while(1){
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
  delay(5000);
  }
  
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
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or try a different address!"));
    Serial.print(F("SensorID was: 0x")); Serial.println(bmp.sensorID(),16);
    Serial.print(F("\tID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n"));
    Serial.print(F("\tID of 0x56-0x58 represents a BMP 280,\n"));
    Serial.print(F("\tID of 0x60 represents a BME 280.\n"));
    Serial.print(F("\tID of 0x61 represents a BME 680.\n"));
    while (1) delay(10);
  }
  //Config the found BMP280 sensor
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,    
                  Adafruit_BMP280::SAMPLING_X2,     
                  Adafruit_BMP280::SAMPLING_X16,    
                  Adafruit_BMP280::FILTER_X16,      
                  Adafruit_BMP280::STANDBY_MS_500); 
  
  for (int thisReading = 0; thisReading < numReadings_sensor_1; thisReading++) { // initialize all the readings of analog sensor to 0:
    readings_sensor_1[thisReading] = 0;
  }
  for (int thisReading = 0; thisReading < numReadings_sensor_2; thisReading++) { // initialize all the readings of analog sensor to 0:
    readings_sensor_2[thisReading] = 0;
  }
  for (int thisReading = 0; thisReading < numReadings_sensor_3; thisReading++) { // initialize all the readings of analog sensor to 0:
    readings_sensor_3[thisReading] = 0;
  }
  for (int thisReading = 0; thisReading < numReadings_sensor_4; thisReading++) { // initialize all the readings of analog sensor to 0:
    readings_sensor_4[thisReading] = 0;
  }
  for (int thisReading = 0; thisReading < numReadings_sensor_5; thisReading++) { // initialize all the readings of analog sensor to 0:
    readings_sensor_5[thisReading] = 0;
  }
  Serial.println("Aight lets cook");
  Serial.println(WiFi.getTxPower());
}

void loop(){

  unsigned long current_time = millis();  //Check if sensors have to be read
  if (current_time - last_PPG_sensor_readout > PPG_SENSOR_READOUT_TIME_MS){
    readPPG = true;
    last_PPG_sensor_readout = current_time;
  }
  if (current_time - last_TEMP_HUM_sensor_readout > TEMP_HUM_SENSOR_READOUT_TIME_MS){
    readTEMP_HUM = true;
    last_TEMP_HUM_sensor_readout = current_time;
  }
  if (current_time - last_LIGHT_sensor_readout > LIGHT_SENSOR_READOUT_TIME_MS){
    readLIGHT = true;
    last_LIGHT_sensor_readout = current_time;
  }
  if (current_time - last_ANALOG_TEMP_sensor_1_readout > ANALOG_TEMP_SENSOR_1_READOUT_TIME_MS){
    readAnalog_TEMP_1 = true;
    last_ANALOG_TEMP_sensor_1_readout = current_time;
  }
  if (current_time - last_ANALOG_TEMP_sensor_2_readout > ANALOG_TEMP_SENSOR_2_READOUT_TIME_MS){
    readAnalog_TEMP_2 = true;
    last_ANALOG_TEMP_sensor_2_readout = current_time;
  }  
  if (current_time - last_ANALOG_TEMP_sensor_3_readout > ANALOG_TEMP_SENSOR_4_READOUT_TIME_MS){
    readAnalog_TEMP_3 = true;
    last_ANALOG_TEMP_sensor_3_readout = current_time;
  }  
  if (current_time - last_ANALOG_TEMP_sensor_4_readout > ANALOG_TEMP_SENSOR_4_READOUT_TIME_MS){
    readAnalog_TEMP_4 = true;
    last_ANALOG_TEMP_sensor_4_readout = current_time;
  }  
   if (current_time - last_ANALOG_TEMP_sensor_5_readout > ANALOG_TEMP_SENSOR_5_READOUT_TIME_MS){
    readAnalog_TEMP_5 = true;
    last_ANALOG_TEMP_sensor_5_readout = current_time;
  }  
  if (current_time - last_blink > LED_DELAY){
    readLed = true;
    last_blink = current_time;
  } 

  if(readPPG){
    readPPG = false;
    tcaselect(PPG_SENSOR);
    auto sample = PPG_sensor.readSample(1000); // De 1000 is de timeout in ms, niet het aantal samples!
    if (sample.valid){  //Check if the sample is valid, then proceed
      uint32_t ir_value = sample.ir;
      uint32_t red_value = sample.red;
      Serial.printf("IR: %d, Red: %d\n", ir_value, red_value);
    }
    else{
      Serial.println("Sample not valid, probably the timeout might be too short!");
    }
  }
  
  if(readTEMP_HUM){  
    readTEMP_HUM = false;
    tcaselect(TEMP_HUM_SENSOR);
    float temp=bmp.readTemperature();
    float pressure=bmp.readPressure();
    float altitude=bmp.readAltitude(1013.25);

    //Serial.printf("Temperature: %f, Pressure: %f, Altitude: %f\n", temp, pressure, altitude);
    X.addField("Temp", temp);
    X.addField("Pressure", pressure);
    X.addField("Altitude", altitude);
  }
  
  if(readLIGHT){
    readLIGHT = false;
    tcaselect(LIGHT_SENSOR);
    BH1750.start();
    float lux=BH1750.getLux();
    //Serial.printf("Light: %f\n", lux);
    X.addField("Lux", lux);
  }
  
  if(readAnalog_TEMP_1){
    readAnalog_TEMP_1 = false;
        // subtract the last reading:
    total_sensor_1 = total_sensor_1 - readings_sensor_1[readIndex_sensor_1];
    // read from the sensor:
    readings_sensor_1[readIndex_sensor_1] = adc1.readVoltage();
    // add the reading to the total:
    total_sensor_1 = total_sensor_1 + readings_sensor_1[readIndex_sensor_1];
    // advance to the next position in the array:
    readIndex_sensor_1 = readIndex_sensor_1 + 1;

    // if we're at the end of the array...
    if (readIndex_sensor_1 >= numReadings_sensor_1) {
      // ...wrap around to the beginning:
      readIndex_sensor_1 = 0;
    }

    // calculate the average:
    average_Vout_sensor_1 = total_sensor_1 / numReadings_sensor_1;
    R_temp_sensor_1 = R2 / ((Vin_sensor_1/average_Vout_sensor_1) - 1);
    //Serial.printf("Vout: %f, R1: %f\n", average_Vout, R1);
    X.addField("temp weerstand 1", R_temp_sensor_1);
  }

  if(readAnalog_TEMP_2){
    readAnalog_TEMP_2 = false;
        // subtract the last reading:
    total_sensor_2 = total_sensor_2 - readings_sensor_2[readIndex_sensor_2];
    // read from the sensor:
    readings_sensor_2[readIndex_sensor_2] = adc2.readVoltage();
    // add the reading to the total:
    total_sensor_2 = total_sensor_2 + readings_sensor_2[readIndex_sensor_2];
    // advance to the next position in the array:
    readIndex_sensor_2 = readIndex_sensor_2 + 1;

    // if we're at the end of the array...
    if (readIndex_sensor_2 >= numReadings_sensor_2) {
      // ...wrap around to the beginning:
      readIndex_sensor_2 = 0;
    }

    // calculate the average:
    average_Vout_sensor_2 = total_sensor_2 / numReadings_sensor_2;
    R_temp_sensor_2 = R2 / ((Vin_sensor_2/(average_Vout_sensor_2)) - 1);
    //Serial.printf("Vout: %f, R1: %f\n", average_Vout, R1);
    //T_temp_sensor_2 = (R_temp_sensor_2/0.269) - (119.398/0.269);
    X.addField("temp weerstand 2", R_temp_sensor_2);
  }

  if(readAnalog_TEMP_3){
    readAnalog_TEMP_3 = false;
        // subtract the last reading:
    total_sensor_3 = total_sensor_3 - readings_sensor_3[readIndex_sensor_3];
    // read from the sensor:
    readings_sensor_3[readIndex_sensor_3] = adc3.readVoltage();
    // add the reading to the total:
    total_sensor_3 = total_sensor_3 + readings_sensor_3[readIndex_sensor_3];
    // advance to the next position in the array:
    readIndex_sensor_3 = readIndex_sensor_3 + 1;

    // if we're at the end of the array...
    if (readIndex_sensor_3 >= numReadings_sensor_3) {
      // ...wrap around to the beginning:
      readIndex_sensor_3 = 0;
    }

    // calculate the average:
    average_Vout_sensor_3 = total_sensor_3 / numReadings_sensor_3;
    R_temp_sensor_3 = R2 / ((Vin_sensor_3/average_Vout_sensor_3) - 1);
    //Serial.printf("Vout: %f, R1: %f\n", average_Vout, R1);
    //T_temp_sensor_2 = (R_temp_sensor_2/0.269) - (119.398/0.269);
    X.addField("temp weerstand 3", R_temp_sensor_3);

  }

  if(readAnalog_TEMP_4){
    readAnalog_TEMP_4 = false;
        // subtract the last reading:
    total_sensor_4 = total_sensor_4 - readings_sensor_4[readIndex_sensor_4];
    // read from the sensor:
    readings_sensor_4[readIndex_sensor_4] = adc4.readVoltage();
    // add the reading to the total:
    total_sensor_4 = total_sensor_4 + readings_sensor_4[readIndex_sensor_4];
    // advance to the next position in the array:
    readIndex_sensor_4 = readIndex_sensor_4 + 1;

    // if we're at the end of the array...
    if (readIndex_sensor_4 >= numReadings_sensor_4) {
      // ...wrap around to the beginning:
      readIndex_sensor_4 = 0;
    }

    // calculate the average:
    average_Vout_sensor_4 = total_sensor_4 / numReadings_sensor_4;
    R_temp_sensor_4 = R2 / ((Vin_sensor_4/average_Vout_sensor_4) - 1);
    //Serial.printf("Vout: %f, R1: %f\n", average_Vout, R1);
    //T_temp_sensor_2 = (R_temp_sensor_2/0.269) - (119.398/0.269);
    X.addField("temp weerstand 4", R_temp_sensor_4);
  }

  if(readAnalog_TEMP_5){
    readAnalog_TEMP_5 = false;
        // subtract the last reading:
    total_sensor_5 = total_sensor_5 - readings_sensor_5[readIndex_sensor_5];
    // read from the sensor:
    readings_sensor_5[readIndex_sensor_5] = adc5.readVoltage();
    // add the reading to the total:
    total_sensor_5 = total_sensor_5 + readings_sensor_5[readIndex_sensor_5];
    // advance to the next position in the array:
    readIndex_sensor_5 = readIndex_sensor_5 + 1;

    // if we're at the end of the array...
    if (readIndex_sensor_5 >= numReadings_sensor_5) {
      // ...wrap around to the beginning:
      readIndex_sensor_5 = 0;
    }

    // calculate the average:
    average_Vout_sensor_5 = total_sensor_5 / numReadings_sensor_5;
    R_temp_sensor_5 = R2 / ((Vin_sensor_5/average_Vout_sensor_5) - 1);
    //Serial.printf("Vout: %f, R1: %f\n", average_Vout, R1);
    //T_temp_sensor_2 = (R_temp_sensor_2/0.269) - (119.398/0.269);
    X.addField("temp weerstand 5", R_temp_sensor_5);
  }

    // Write the point to InfluxDB
  if (client.writePoint(X)) {
    //Serial.println("Data sent to InfluxDB successfully!");
    //Serial.println("\tAvailable RAM memory: " + String(esp_get_free_heap_size()) + " bytes");

    strip.setPixelColor(0, strip.Color(255,0,0));
    strip.show();

  } else {
    /*
    strip.setPixelColor(0, strip.Color(0,255,0));
    strip.show();
    */
    strip.clear();
    strip.show();
    Serial.print("InfluxDB write failed: ");
    Serial.println(client.getLastErrorMessage());
  }
  // Clear previous data from the point
  X.clearFields();
}
