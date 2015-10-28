#include<ir.h>
// This enumeration matches the distance of the obstacle
typedef enum {
    nearest = 0,
    near = 1,
    far = 2,
} distance_obstacle;


bool a::init()
{
    p1_wire30.setAsInput();

    return true;
}

bool a::run(void *p)
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