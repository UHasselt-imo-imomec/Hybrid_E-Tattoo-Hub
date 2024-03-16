/*
 * ESP32AnalogRead.cpp
 *
 *  Created on: Apr 10, 2020
 *      Author: hephaestus
 *      https://github.com/madhephaestus/ESPMutexDemo/blob/DSPTest/ESPMutexDemo.ino
*       Modified by Claude Arpin March 3, 2022
*     www.seenov.com
*   ADC2  not working possibly issue attachpin pinNUm >4 ,  5 = ADC2 channel 0
 */

#include "ESP32AnalogRead.h"

ESP32AnalogRead::ESP32AnalogRead(int pinNum) {
	if (!(pinNum < 0)) {  // attach pin 5 = ADC2 chan 0, creates reboot.
		attach(pinNum);
	}
}
void ESP32AnalogRead::attach(int pin) {
	myPin = pin;
	channel = (adc_channel_t) digitalPinToAnalogChannel(myPin);
	attached = true;
}

float ESP32AnalogRead::readVoltage() {
	int mv = readMiliVolts();

	return mv * 0.001;
}

int ESP32AnalogRead::readMiliVolts() {
	if (!attached)
		return 0;
	analogRead(myPin);
	// Configure ADC
	adc_unit_t unit;
	if (myPin <5) {
		adc1_config_width(ADC_WIDTH_12Bit);
		adc1_channel_t chan= ADC1_CHANNEL_0;
		unit=ADC_UNIT_1;
		switch (myPin) {

		case 0:
			chan = ADC1_CHANNEL_0;
			break;
		case 1:
			chan = ADC1_CHANNEL_1;
			break;
		case 2:
			chan = ADC1_CHANNEL_2;
			break;
		case 3:
			chan = ADC1_CHANNEL_3;
			break;
		case 4:
			chan = ADC1_CHANNEL_4;
			break;
		
		}
		adc1_channel = chan;
		adc1_config_channel_atten(chan, ADC_ATTEN_6db);
	} else {
       // adc2_config_width(ADC_WIDTH_12Bit);
		adc2_channel_t chan= ADC2_CHANNEL_0;
		unit=ADC_UNIT_2;
		switch (myPin) {
		case 5:
			chan = ADC2_CHANNEL_0;
			break;
		}
		adc2_channel = chan;
		adc2_config_channel_atten(chan, ADC_ATTEN_6db); 
	}
	// Calculate ADC characteristics i.e. gain and offset factors
	esp_err_t ret;
	ret = esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP);
	esp_adc_cal_characterize(unit,
	ADC_ATTEN_DB_6,
	ADC_WIDTH_BIT_12,
	0,
	&characteristics);

    esp_err_t ret1 = ESP_OK;
	int raw = 0;
	uint32_t voltage = 0;
	// Read ADC and obtain result in mV

	if (unit == ADC_UNIT_1) {
		raw = adc1_get_raw(adc1_channel);
        voltage = esp_adc_cal_raw_to_voltage(raw, &characteristics);
        return voltage;
	} else {
	   ret1 =	adc2_get_raw(ADC2_CHANNEL_0, ADC_WIDTH_BIT_12,  &raw);
      if (ret1== ESP_OK){
              voltage =  esp_adc_cal_raw_to_voltage(raw, &characteristics);
            return voltage;
       }
        return 5000;  // means there is an error, adc2 not yet supported
	}
	
}