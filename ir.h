#ifndef IR_H
#define IR_H
#ifdef _cplusplus
extern "C"{
#endif
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
#include "lpc_timers.h">





class a : public scheduler_task
{
   private : //ports to be used
       GPIO p1_wire30;

    //constructor
    public:
           a(uint8_t kpriority) :
                   scheduler_task("a", 2000, kpriority),
                   p1_wire30(LPC1758_GPIO_Type::P1_30)
            {

            }
		
		bool init();
		bool run(void *);

};

};
#ifdef _cplusplus
}
#endif /*IR_H*/
