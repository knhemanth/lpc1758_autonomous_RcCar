//Divya Editing IR_sharp_sensor.cpp
#include "FreeRTOS.h"   //for adc
#include "semphr.h"
#include "queue.h"
#include "task.h"     //for adc
#include <IR_sharp_sensor.hpp>
#include "adc0.h"

//int IR_sharp_sensor :: reading ;
IR_sharp_sensor ir_sharp(P1_30);

IR_sharp_sensor :: IR_sharp_sensor(LPC1758_GPIO_Type inpin) : sharp_in(inpin)
{
    sharp_in.setAsInput();
    LPC_PINCON->PINSEL3 |=  (3 << 28); // ADC-4 is on P1.30, select this as ADC0.4
    // Initialization :
    reading = 0;
}

int IR_sharp_sensor :: get_ir_reading ()
{
    reading = adc0_get_reading(4); // Read current value of ADC-4

    return reading;
}

void ir_sharp_readings()
{
    static int a;

    a = ir_sharp.get_ir_reading(); //push it to dist_sensor back
}
/*
bool IR_sharp_sensor::init()
{
    p1_wire30.setAsInput();

    return true;
}

bool IR_sharp_sensor::run(void *p)
{
  //read from pin
 int reading = 0;
 distance_obstacle d;

  // Initialization :
  LPC_PINCON->PINSEL3 |=  (3 << 28); // ADC-4 is on P1.30, select this as ADC0.4

  while(1) {
     reading = adc0_get_reading(4); // Read current value of ADC-4
//   printf("\nADC Reading: %d", reading);
    if((reading >= 0) && (reading <= 1023))
    {
         d = far;
    }
    else
    {
     if((reading >= 2048) && (reading <= 4096))
     {
       d = nearest;
     }
     else
     {
     if((reading >= 1024) && (reading <= 2047))
      {
              d = near;
      }
     }
    }

    printf("distance == %d",d);
    delay_ms(1000);
    }
    return true;
    }
};
*/

