/*
 * ultrasonic_sensor.cpp
 *
 *  Created on: Sep 10, 2015
 *      Author: Hemanth K N and Akshay Vijaykumar
 */

#include <stdio.h>
#include "ultrasonic_sensor.hpp"
#include "adc0.h"

/* TogglePower
 * This function allows the caller to turn the sensor on or off.
 * */
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

/* Recalibrate
 * Performs a power cycle of the Sensor
 * */
void UltrasonicSensor::Recalibrate()
{
    TogglePower(OFF);
    TogglePower(ON);
}

/* ReadRange
 * Return the Raw Range value obtained from Sensor
 * */
float UltrasonicSensor::ReadRange(void)
{
    /* The function does the following:
     * Read the ADC Step value for a particular voltage
     * Calculate the voltage value by multiplying with ADC Step Voltage
     * The value is divided by voltage per inch.
     * */
    float usonic_adc_reading = 0;
    float usonic_sensor_reading_v = 0;     // reading of sensor in volts
    float usonic_sensor_reading_inch = 0;     // reading of sensor in inches

    usonic_adc_reading = (float)adc0_get_reading(USONIC_FRONTSENSOR_ADCCHANNEL);
    usonic_sensor_reading_v = ( usonic_adc_reading * ADC_VOLTAGE_PER_STEP );
    usonic_sensor_reading_inch = ( usonic_sensor_reading_v / USONIC_VOLTAGE_PER_INCH );

    if(getFilterUsage())
    {
        /* Send this data to Filter */
        avg_filter.addValue(usonic_sensor_reading_inch);
    }

    return usonic_sensor_reading_inch;
}

/* GetFilteredRangeValue
 * Returns the Filtered range output using an average filter
 * */

float UltrasonicSensor::GetFilteredRangeValue(void)
{
    /* If Filter has been used for the sensor, the output of the filter will be read from here
     * Else, this function returns 0 */

    if(getFilterUsage())
    {
        return avg_filter.getValue();
    }
    return 0;
}
