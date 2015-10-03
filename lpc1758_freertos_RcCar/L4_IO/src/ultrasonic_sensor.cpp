/*
 * ultrasonic_sensor.cpp
 *
 *  Created on: Sep 10, 2015
 *      Author: Hemanth K N
 */

#include <stdio.h>
#include "ultrasonic_sensor.hpp"
#include "adc0.h"


void UltrasonicSensor::TogglePower(bool power)
{

    if ( power )  // If power on is requested and it is not already on
    {
        GpioPower.setHigh();
    }

    else         // If power off is requested and it is not already off
    {
        GpioPower.setLow();
    }

}

void UltrasonicSensor::Recalibrate()
{
    TogglePower(OFF);
    TogglePower(ON);
}

float UltrasonicSensor::GetRangeValue(void)
{
    float usonic_adc_reading = 0;
    float usonic_sensor_reading_v = 0;     // reading of sensor in volts
    float usonic_sensor_reading_inch = 0;     // reading of sensor in inches

    usonic_adc_reading = (float)adc0_get_reading(USONIC_FRONTSENSOR_ADCCHANNEL);
    usonic_sensor_reading_v = ( usonic_adc_reading * ADC_VOLTAGE_PER_STEP );
    usonic_sensor_reading_inch = ( usonic_sensor_reading_v / USONIC_VOLTAGE_PER_INCH );

    return usonic_sensor_reading_inch;

}
