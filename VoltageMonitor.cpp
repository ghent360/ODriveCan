
/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
*/
#include <ADC.h>
#include <ADC_util.h>
#include <Snooze.h>
#include "CanInterface.h"

static const int batteryVoltagePin = A17; // ADC0

static ADC adc; // adc object;
static SnoozeBlock config;

void initVoltageMonitor() {
    pinMode(batteryVoltagePin, INPUT);

    adc.adc0->setAveraging(16); // set number of averages
    adc.adc0->setResolution(16); // set bits of resolution

    adc.adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_LOW_SPEED); // change the conversion speed
    adc.adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_LOW_SPEED); // change the sampling speed
}

float readBatteryVoltage() {
    // Single reads
    int value = adc.adc0->analogRead(batteryVoltagePin);
    float vIn = value*3.3*(82+22)/(22*adc.adc0->getMaxValue());

    // Print errors, if any.
    if(adc.adc0->fail_flag != ADC_ERROR::CLEAR) {
      Serial.print("ADC0: ");
      Serial.println(getStringADCError(adc.adc0->fail_flag));
      return -1;
    }
    return vIn;
}

void lowPowerMode() {
    canSleep();
    Snooze.hibernate(config);
}