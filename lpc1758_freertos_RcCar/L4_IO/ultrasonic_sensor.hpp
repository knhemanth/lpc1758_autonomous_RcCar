/*
 * ultrasonic_sensor.hpp
 *
 *  Created on: Sep 10, 2015
 *      Author: Hemanth K N
 */

#ifndef L4_IO_ULTRASONIC_SENSOR_HPP_
#define L4_IO_ULTRASONIC_SENSOR_HPP_

#include "gpio.hpp"
#include "LPC17xx.h"    // For Pin Select
#include "singleton_template.hpp"  // Singleton Template


#define USONIC_SUPPLY_VCC               ( 3.3 )
#define USONIC_FRONTSENSOR_ADCCHANNEL   ( 4 )
#define USONIC_VOLTAGE_PER_INCH         ( (float)( (float)USONIC_SUPPLY_VCC/512) )    // From datasheet in v
#define ADC04_PINSELECT_VALUE           (  0x00000003 )
#define ADC04_PINSELECT_SHIFT           ( 28 )
#define ADC_VOLTAGE_PER_STEP            ( (float)( (float)3/4096 ) )    // 12-bit ADC has 4096 steps across 3000mVolts

#define OFF                             ( false )
#define ON                              ( true )

class UltrasonicSensor
{
    private:
        GPIO GpioPower;     /* GPIO pin used as the Power Supply to the Ultrasonic Sensor */

    protected:

        UltrasonicSensor(LPC1758_GPIO_Type Power_Pin):
            GpioPower(Power_Pin)
        {
            /* Configuring the GpioPower pin as Output */
            GpioPower.setAsOutput();
        }

        virtual ~UltrasonicSensor()
        {
            /* Nothing to do*/
        }

    public:
        virtual void TogglePower(bool power);             // Power On/Off the sensor
        virtual void Recalibrate();                      // Recalibrate the sensor [ power cycle ]
        virtual float GetRangeValue(void);     // Return range from the sensor
};


class UsonicFrontSensor : public UltrasonicSensor, public SingletonTemplate<UsonicFrontSensor>
{
    private:
        UsonicFrontSensor():
            UltrasonicSensor(P2_0)   // Interface front sensor power to PIO2.0
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

#endif /* L4_IO_ULTRASONIC_SENSOR_HPP_ */
