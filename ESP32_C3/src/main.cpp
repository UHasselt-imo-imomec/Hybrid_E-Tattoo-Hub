#include "Arduino.h"
#include <ESP32AnalogRead.h>
ESP32AnalogRead adc;

int analogPin = 0;
float Vin = 3.3;
float Vout = 0;
float R1 = 0;
float R2 = 220;
float buffer = 0;
float VoutOverVin = 0;

void setup(){
	adc.attach(analogPin);
	Serial.begin(115200);
}

void loop(){
  Vout = adc.readVoltage();
  R1 = R2 * ((Vin/Vout) - 1);
  Serial.print("Vout: ");
  Serial.println(Vout, 4);
  Serial.print("R1: ");
  Serial.println(R1);
  delay(1000);
}