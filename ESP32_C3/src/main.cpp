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

//Timings for sensor readout:
#define ANALOG_TEMP_SENSOR_1_READOUT_TIME_MS 100
#define ANALOG_TEMP_SENSOR_2_READOUT_TIME_MS 100
#define LED_DELAY 1000
unsigned long last_ANALOG_TEMP_sensor_1_readout = 0;
unsigned long last_ANALOG_TEMP_sensor_2_readout = 0;
unsigned long last_blink = 0;
bool readPPG = false;
bool readTEMP_HUM = false;
bool readLIGHT = false;
bool readAnalog_TEMP_1 = false;
bool readAnalog_TEMP_2 = false;
bool readLed = false;


// analoge sensor
// Define the number of samples to keep track of. The higher the number, the
// more the readings will be smoothed, but the slower the output will respond to
// the input.
const int numReadings_sensor_1 = 100;
int analogPin_sensor_1 = 2;
int readIndex_sensor_1 = 0;            // the index of the current reading
float readings_sensor_1[numReadings_sensor_1];  // the readings from the analog input
float total_sensor_1 = 0;              // the running total
float average_Vout_sensor_1 = 0;       // the average voltage
float Vin_sensor_1 = 3.3;
float R_temp_sensor_1 = 0;
float R2 = 220;
ESP32AnalogRead adc1;

const int numReadings_sensor_2 = 100;
int analogPin_sensor_2 = 3;
int readIndex_sensor_2 = 0;            // the index of the current reading
float readings_sensor_2[numReadings_sensor_2];  // the readings from the analog input
float total_sensor_2 = 0;              // the running total
float average_Vout_sensor_2 = 0;       // the average voltage
float Vin_sensor_2 = 3.3;
float R_temp_sensor_2 = 0;
ESP32AnalogRead adc2;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, 7, NEO_GRB + NEO_KHZ800);

void setup(){
  
  strip.begin();
  strip.setBrightness(50);
  strip.show(); // Initialize all pixels to 'off'

  delay(5000);    //Delay to let Serial Monitor catch up (Because of CDCBoot)

  adc1.attach(analogPin_sensor_1);
  adc2.attach(analogPin_sensor_2);

  Serial.begin(115200);
  Serial.println("Initializing");

  WiFi.mode(WIFI_STA);
  wifiMulti.addAP(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to wifi");
  WiFi.setTxPower(WIFI_POWER_2dBm);
  Serial.print(WiFi.getTxPower());

  while (wifiMulti.run() != WL_CONNECTED) {
    Serial.print(WiFi.getTxPower());
    delay(100);
  }

  Serial.println();

  if(WiFi.isConnected()){
    strip.setPixelColor(0, strip.Color(0,0,255));
    strip.show();
    delay(1000);
    strip.clear();
    strip.show();
    Serial.print(WiFi.getTxPower());
  }

  timeSync(TZ_INFO, "pool.ntp.org", "time.nis.gov");
  strip.clear();
  strip.show();
  if (client.validateConnection()) {
    Serial.print("Connected to InfluxDB_1: ");
    Serial.println(client.getServerUrl());
  } else {
    Serial.print("InfluxDB connection failed: ");
    Serial.println(client.getLastErrorMessage());
  }
  Serial.println("\tAvailable RAM memory: " + String(esp_get_free_heap_size()) + " bytes");
  
  // Set write options for batching and precision
  client.setWriteOptions(
      WriteOptions()
          .writePrecision(WritePrecision::MS)
          .batchSize(200)
          .bufferSize(1000)
          .flushInterval(60)
  );
  
  // Set HTTP options for the client
  client.setHTTPOptions(
      HTTPOptions().connectionReuse(true)
  );  
  
  for (int thisReading = 0; thisReading < numReadings_sensor_1; thisReading++) { // initialize all the readings of analog sensor to 0:
    readings_sensor_1[thisReading] = 0;
  }
  for (int thisReading = 0; thisReading < numReadings_sensor_2; thisReading++) { // initialize all the readings of analog sensor to 0:
    readings_sensor_2[thisReading] = 0;
  }

  Serial.println("Aight lets cook");
  Serial.println(WiFi.getTxPower());
}

void loop(){

  unsigned long current_time = millis();  //Check if sensors have to be read
  if (current_time - last_ANALOG_TEMP_sensor_1_readout > ANALOG_TEMP_SENSOR_1_READOUT_TIME_MS){
    readAnalog_TEMP_1 = true;
    last_ANALOG_TEMP_sensor_1_readout = current_time;
  }
  if (current_time - last_ANALOG_TEMP_sensor_2_readout > ANALOG_TEMP_SENSOR_2_READOUT_TIME_MS){
    readAnalog_TEMP_2 = true;
    last_ANALOG_TEMP_sensor_2_readout = current_time;
  }
  if (current_time - last_blink > LED_DELAY){
    readLed = true;
    last_blink = current_time;
  }  

  if(client.isBufferEmpty()){
    strip.setPixelColor(0, strip.Color(255,0,255));
    strip.show();
    readLed = false;
  } else {
    if(readLed){
    readLed = false;
    strip.clear();
    strip.show();
    }
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
    Serial.printf("Vout: %f, R1: %f\n", average_Vout_sensor_1, R_temp_sensor_1);
    X.addField("temp weerstand 1", R_temp_sensor_1);
    if (client.writePoint(X)) {
    } else {
      strip.setPixelColor(0, strip.Color(0,255,0));
      strip.show();
      delay(1000);
      Serial.print("InfluxDB write failed: ");
      Serial.println(client.getLastErrorMessage());
    }
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
    Serial.printf("Vout: %f, R2: %f\n", average_Vout_sensor_2, R_temp_sensor_2);
    //T_temp_sensor_2 = (R_temp_sensor_2/0.269) - (119.398/0.269);
    X.addField("temp weerstand 2", R_temp_sensor_2);
    if (client.writePoint(X)) {
    } else {
      strip.setPixelColor(0, strip.Color(0,255,0));
      strip.show();
      delay(1000);
      Serial.print("InfluxDB write failed: ");
      Serial.println(client.getLastErrorMessage());
    }
  }

  // Clear previous data from the point
  X.clearFields();
}