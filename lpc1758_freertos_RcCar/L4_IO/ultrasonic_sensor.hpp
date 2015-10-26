/*
 * ultrasonic_sensor.hpp
 *
 *  Created on: Sep 10, 2015
 *      Author: Hemanth K N and Akshay Vijaykumar
 */

#ifndef L4_IO_ULTRASONIC_SENSOR_HPP_
#define L4_IO_ULTRASONIC_SENSOR_HPP_

/* MODULE USAGE :
 * In the 10hz periodic task, Call US_FRONT.ReadRange();
 * This is followed by US_FRONT.GetFilteredRangeValue());
 * ReadRange would read the current value while GetFilteredRangeValue would
 * provide a filtered value based on the past 5 range readings.
 *
 * Make sure to include ultrasonic_sensor.hpp in the period_callbacks.cpp file
 * */


#include "gpio.hpp"
#include "LPC17xx.h"               // For Pin Select
#include "singleton_template.hpp"  // Singleton Template
#include "circular_buffer.hpp"
#include "FreeRTOS.h"
#include "semphr.h"
#include "projdefs.h"
#include "file_logger.h"


/* Custom Debug Print Function */
#if 0
#define PRINTME(...) LOG_DEBUG(__VA_ARGS__)
#else
#define PRINTME(...)
#endif

#define USONIC_SUPPLY_VCC               ( 3.3 )
#define USONIC_FRONTSENSOR_ADCCHANNEL   ( 4 )
#define USONIC_VOLTAGE_PER_INCH         ( (float)( (float)USONIC_SUPPLY_VCC/512) )    // From datasheet in v
#define ADC04_PINSELECT_VALUE           (  0x00000003 )
#define ADC04_PINSELECT_SHIFT           ( 28 )
#define ADC_VOLTAGE_PER_STEP            ( (float)( (float) 3/4096 ) )    // 12-bit ADC has 4096 steps across 3000mVolts

#define OFF                             ( false )
#define ON                              ( true )

#define OVERWRITE_OLD_DATA              ( true )

#define FILTER_ACCESS_TIMEOUT           ( 1 )

/* Sensor Filter Template
 * Performs Average Filtering.
 * This class can be inherited with other sensor classes for filtering
 * */

template <typename INTYPE, typename OUTTYPE>
class Sensor_Filter
{
    private:
        CircularBuffer <INTYPE> cBuf; /* Circular Buffer holding Sensor Values */
        double currentAverage;
        SemaphoreHandle_t filterAccess;
        bool stableStream;

        /* This function calculates the average of values in cBuf */
        void updateAverage( INTYPE data)
        {
            double tempVal = 0;
            float mulFactor = 0;
            INTYPE oldElement = 0;

            if( !stableStream )
            {
                /* Add data to Buffer */
                cBuf.push_back(data);
                mulFactor = 1/(double)(cBuf.size());

                /* If number of elements in the buffer < total buffer capacity
                 * Calculate the average using loops */

                for(int i = 0; i < (int)cBuf.size(); i++)
                {
                    tempVal += cBuf[i];
                }

                tempVal = tempVal * mulFactor;

                currentAverage = tempVal;

                stableStream = (cBuf.size() == cBuf.capacity());
            }
            else
            {
                /* Efficient way of calculating average in case the buffer is full
                 * Every calculation will update the current average value.
                 * To get new average:
                 * new_average =
                 *         current_average +
                 *         ( new_element - oldest_element ) * mulFactor
                 *
                 * The CircularBuffer at this point is Full
                 * */

                oldElement = cBuf.peek_front();
                /* Adding new data Element */

                cBuf.push_back(data, OVERWRITE_OLD_DATA);

                /* We now have a pointer to the old element in the buffer */
                tempVal = currentAverage;

                tempVal = tempVal + ( (  data - oldElement ) / ( cBuf.capacity() ) );

                currentAverage = tempVal;

            }

        }

    public:

        Sensor_Filter(uint32_t bufSize):
            cBuf(bufSize),
            currentAverage(0),
            stableStream(false)

        {
            filterAccess = xSemaphoreCreateMutex();
            /* Nothing to be done */
        }

        void addValue(INTYPE data)
        {
            if( xSemaphoreTake(filterAccess, FILTER_ACCESS_TIMEOUT) )
            {
                /*  Push Value into FIFO and update Average */
                updateAverage(data);
                xSemaphoreGive(filterAccess);
            }
            else
            {
                printf("Semaphore not available for writing to filter\n");
            }
        }

        OUTTYPE getValue(void)
        {
            OUTTYPE tempVar = 0;

            if( xSemaphoreTake(filterAccess, FILTER_ACCESS_TIMEOUT) )
            {
                /* Return the current Average Value */
                tempVar = static_cast<OUTTYPE> (currentAverage);

                xSemaphoreGive(filterAccess);
            }

            else
            {
                printf("Semaphore not available for reading from filter\n");
            }

            return tempVar;


        }
};

/* Ultrasonic Sensor Class
 * */
class UltrasonicSensor
{
    private:
        GPIO GpioPower;     /* GPIO pin used as the Power Supply to the Ultrasonic Sensor */
        Sensor_Filter <double, double> avg_filter; /* Sensor Filter applied to Ultrasonic Sensor */
        bool isFilterUsed; /* Holds information of usage of a filter */

    protected:

        UltrasonicSensor(LPC1758_GPIO_Type Power_Pin, bool filterOpt = false, int filterSize = 0):
            GpioPower(Power_Pin),
            avg_filter(filterSize),
            isFilterUsed(filterOpt)
        {
            /* Configuring the GpioPower pin as Output */
            GpioPower.setAsOutput();
        }


    public:
        /* Status Variables */

        inline bool getFilterUsage(void)
        {
            return isFilterUsed;
        }

        virtual void TogglePower(bool power);             // Power On/Off the sensor
        virtual void Recalibrate();                       // Recalibrate the sensor [ power cycle ]
        virtual float ReadRange(void);                    // Return range from the sensor
        virtual float GetFilteredRangeValue(void);        // Return range value from the sensor

};

/* Ultrasonic Sensor at Front
 * Uses P2_0 to Power it on.
 * Uses ADC0.4.
 * */
class UsonicFrontSensor : public UltrasonicSensor, public SingletonTemplate<UsonicFrontSensor>
{
    private:
        UsonicFrontSensor():
            UltrasonicSensor(P2_0, true, 10)   /* Interface front sensor power to PIO2.0; Filtering True; 10 Elements */
            {
                init();
            }

        void init()
        {
            /*
             *  We have interfaced this sensor to ADC0.4
             *  Setting the corresponding PinMux
             */

            LPC_PINCON->PINSEL3 |= (ADC04_PINSELECT_VALUE << ADC04_PINSELECT_SHIFT);

            // Power on the sensor
            TogglePower(ON);

        }

        friend class SingletonTemplate<UsonicFrontSensor>;

};

// Macro to get Singleton Instance
#define US_FRONT UsonicFrontSensor::getInstance()


#endif /* L4_IO_ULTRASONIC_SENSOR_HPP_ */
