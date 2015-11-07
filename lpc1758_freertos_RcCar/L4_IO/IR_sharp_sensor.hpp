
//#include <3ultrasonic_sensor_interrupts.hpp>
#include "lpc_sys.h"
#include "tasks.hpp"
#include "examples/examples.hpp"
#include "tasks.hpp"
#include "examples/examples.hpp"
#include "FreeRTOS.h"
#include "stdio.h"
#include "utilities.h"
#include "printf_lib.h"
#include "string.h"
#include "io.hpp"
#include "stdio.h"
#include "eint.h"
#include "gpio.hpp"
#include "queue.h"
#include "rtc.h"
#include "LPC17xx.h"
#include "lpc_timers.h"
#include "adc0.h"

class IR_sharp_sensor
{
   private : //ports to be used
      // GPIO p1_wire30;
       GPIO sharp_in;
       int reading;

    //constructor
    public:
           IR_sharp_sensor(LPC1758_GPIO_Type in);

           /*(uint8_t kpriority) :
                   scheduler_task("IR", 2000, kpriority),
                   p1_wire30(LPC1758_GPIO_Type::P1_30)*/

           int get_ir_reading();
};


