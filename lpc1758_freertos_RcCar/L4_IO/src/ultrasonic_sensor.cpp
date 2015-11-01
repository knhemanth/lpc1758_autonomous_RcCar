/*
 * ultrasonic_sensor.cpp
 *
 *  Created on: Sep 10, 2015
 *      Author: Hemanth K N and Akshay Vijaykumar
 */

#include <stdio.h>
#include "ultrasonic_sensor.hpp"
#include "adc0.h"
#include "utilities.h" // for TIME_US() in PING
#include "eint.h"   //for EINT3XXXXXX used in PING

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


//UltraSonic ping 4-pin sensor Divya editing

SemaphoreHandle_t Ultra_Sonic_4ping :: Trig_Sem;//indicating the static variables created in .hpp file belong to this class
uint64_t  Ultra_Sonic_4ping :: up_time;
uint64_t Ultra_Sonic_4ping :: down_time;
uint32_t Ultra_Sonic_4ping :: diff_time;
float  Ultra_Sonic_4ping :: distance_value;
Sensor_Filter <double, double> Ultra_Sonic_4ping:: avg_filter;
QueueHandle_t Ultra_Sonic_4ping:: xQueue1;//xQueue2,xQueue3;


//Constructor for the 4-pin ping sensor
Ultra_Sonic_4ping :: Ultra_Sonic_4ping( LPC1758_GPIO_Type pTrig_out_pin,  LPC1758_GPIO_Type pEcho_in_pin)://, uint32_t pfilterSize = 5):
    trig_out(pTrig_out_pin),
    echo_in(pEcho_in_pin)

{
   Trig_Sem = xSemaphoreCreateBinary();  //first give the Sem at initialization
   xSemaphoreGive(Trig_Sem);             //send trig() will first check if its taken
   eint3_enable_port2(1,eint_rising_edge,echo_high_callback);
   eint3_enable_port2(1,eint_falling_edge,echo_low_callback);
   trig_out.setAsOutput();
   echo_in.setAsInput();
   xQueue1 = xQueueCreate(2,sizeof(float));
}

bool Ultra_Sonic_4ping ::send_trig()
{
    if( xSemaphoreTake(Trig_Sem,0))
    {
        //reset the GPIO pins
        trig_out.set(false);
        delay_us(5);

        trig_out.setHigh();
        delay_us(10);
       trig_out.setLow();

    }
    return true;
}

void Ultra_Sonic_4ping :: echo_high_callback()
{

    up_time = sys_get_uptime_us();
}

void Ultra_Sonic_4ping :: echo_low_callback()
{
    down_time = sys_get_uptime_us();

    //Calculate the time of the ECHO pulse

    diff_time = down_time - up_time;

    distance_value = (float)(diff_time)*(1.0/58);  //Distance in cms

    xQueueSendFromISR(xQueue1,&distance_value,0);

    xSemaphoreGiveFromISR(Trig_Sem,false);

}

float Ultra_Sonic_4ping :: ping_get_from_filter()
{
    float d = 0.0;

    d = avg_filter.getValue();

    return d;
}

bool Ultra_Sonic_4ping :: recieve_from_queue()
{
    //this will fetch the value from the queue and put it in a buffer(pass-by-ref),0 delay / ticks
    xQueueReceive( xQueue1, &buff_for_recieve, 0);
    return true;
}

float Ultra_Sonic_4ping :: get_buffer_value()
{
    return buff_for_recieve;
}

void Ultra_Sonic_4ping :: add_queue_value_to_filter(float a)
{
    avg_filter.addValue(a);
}
