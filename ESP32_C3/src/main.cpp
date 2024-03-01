#include "Arduino.h"
#include <ESP32AnalogRead.h>
ESP32AnalogRead adc;

int analogPin = 0;
int Vin = 3.3;
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
  buffer = ((Vin/Vout) - 1);
  R1 = R2*buffer;
  Serial.print("Vout: ");
  Serial.println(Vout, 4);
  Serial.print("buffer: ");
  Serial.println(buffer, 4);
  Serial.print("R1: ");
  Serial.println(R1, 1);
  delay(100);
}