#include "tasks.hpp"
#include <stdio.h>
#include <FreeRTOS.h>
#include "lpc_timers.h"
#include "gpio.hpp"
#include "io.hpp"
#include "task.h"
#include "utilities.h"
#include "lpc_pwm.hpp"
#include "printf_lib.h"
//#include "proj_test.hpp"
//#include "can_common.hpp"
//#include "examples/examples.hpp"
//#include "motor_control.hpp"
#include "eint.h"
bool obstacle=true;


enum direction{
right,
right_l,
centre,
left_l,
left}dir;



/**
 * The main() creates tasks or "threads".  See the documentation of scheduler_task class at scheduler_task.hpp
 * for details.  There is a very simple example towards the beginning of this class's declaration.
 *
 * @warning SPI #1 bus usage notes (interfaced to SD & Flash):
 *      - You can read/write files from multiple tasks because it automatically goes through SPI semaphore.
 *      - If you are going to use the SPI Bus in a FreeRTOS task, you need to use the API at L4_IO/fat/spi_sem.h
 *
 * @warning SPI #0 usage notes (Nordic wireless)
 *      - This bus is more tricky to use because if FreeRTOS is not running, the RIT interrupt may use the bus.
 *      - If FreeRTOS is running, then wireless task may use it.
 *        In either case, you should avoid using this bus or interfacing to external components because
 *        there is no semaphore configured for this bus and it should be used exclusively by nordic wireless.
 */


extern "C"
void EINT3_IRQHandle(void)
{
    u0_dbg_put("\nNew interrupt handler \n");
        if(LPC_GPIOINT->IO0IntStatR & (1<<1))
        {
          //  xSemaphoreGiveFromISR(signal1,NULL);
            LPC_GPIOINT->IO0IntClr=(1<<1);
            u0_dbg_put("Signal 1 sent \n");
        }
        if(LPC_GPIOINT->IO0IntStatR & (1<<0))
        {
          //  xSemaphoreGiveFromISR(signal2,NULL);
            LPC_GPIOINT->IO0IntClr=(1<<0);
            //u0_dbg_put("Signal 2 sent \n");
        }

    LPC_GPIOINT->IO0IntClr = 0xFFFFFFFF;             // Clearing all interrupts as a safe side.
}




void call()
{

 printf("\n\n Intrrupt clled \n");

}












void motor_task1(void* p) {
float middle=8.3;
float less_right=6.5;
float ext_right=4.5;
float less_left=10.3;
float ext_left=12.0;
int del=1000;

    PWM pwmServo(PWM::pwm1,50); //As per waveform and real testing(percent range - 5.0(right) - 7.5(center) - 10(left))
    pwmServo.set(middle);

    while(1)
    {
 printf("centre\n");
        dir=centre;
        delay_ms(del);
        printf("centre\n");
        pwmServo.set(less_right);
      dir=right_l;
       delay_ms(del);
        pwmServo.set(ext_right);
      dir=right;
        delay_ms(del);
        pwmServo.set(less_right);
        dir=right_l;
         delay_ms(del);
         pwmServo.set(middle);
         dir=centre;
        delay_ms(del);
         pwmServo.set(less_left);
         dir=left_l;
         delay_ms(del);
         printf("left\n");
         pwmServo.set(ext_left);
         dir=left;
         delay_ms(del);
        pwmServo.set(less_left);
         dir=left_l;
         delay_ms(del);
         pwmServo.set(middle);
    }
}

void sensor_task1(void* p) {
    uint32_t timer_val=0;
    float dist_val=0;
    bool flag_u=true;
    bool flag_u_high=false;
    LE.init();


    // setting 0.1 as echo pin... (input)
    LPC_GPIO0 -> FIODIR &= ~(1 << 1);

    //trigger pin be 0.0.. output
    LPC_GPIO0 -> FIODIR |= (1 << 0);


    //GPIO p2_wire1(P2_3);
    //GPIO trigger_pin(P2_1);


    //p2_wire1.setAsInput();
    //trigger_pin.setAsOutput();
    lpc_timer_enable(lpc_timer0,1);
    LE.off(1);

    while(1) {
        if(flag_u) {
            printf("in if 1\n");
            //trigger_pin.setAsOutput();
            LPC_GPIO0 -> FIODIR |= (1 << 0);
            LPC_GPIO0->FIOSET = (1 << 0);

          //  trigger_pin.setHigh();
            delay_us(11);
            LPC_GPIO0->FIOCLR = (1 << 0);

          //  trigger_pin.setLow();
            LPC_GPIO0 -> FIODIR &= ~(1 << 1);
           // p2_wire1.setAsInput();
            flag_u = false;
        }

        if( (LPC_GPIO0->FIOPIN & (1 << 1)) && !flag_u_high) {
            printf("in if 2\n");
            lpc_timer_enable(lpc_timer0,1);
            lpc_timer_set_value(lpc_timer0,0);
            flag_u_high = true;
        }
        else if(!(LPC_GPIO0->FIOPIN & (1 << 1)) && flag_u_high){
            printf("in if 3\n");
            timer_val = lpc_timer_get_value(lpc_timer0);
            printf("timer val is %d\n",timer_val);
           // vTaskDelay(1000);
            dist_val = 340.0 * (float)timer_val * 0.0001/2.0;
            if(dist_val<150) {
                obstacle = true;
                printf("\n\n direvtion is %d:\n",dir);
                LE.on(1);
            }
            else {
                obstacle = false;
                LE.off(1);
            }
           // printf("Distance value: %f\n",dist_val);
            flag_u = true;
            flag_u_high = false;
        }

        if(lpc_timer_get_value(lpc_timer0) > 53000 && !flag_u) {
            printf("in if 5\n");
            obstacle = false;
            LE.off(1);
            flag_u = true;
            flag_u_high = false;
            lpc_timer_enable(lpc_timer0,1);
            lpc_timer_set_value(lpc_timer0,0);
         //   printf("No reflected pulse!\n");
        }

    }

}





int main(void)
{
    //LPC_GPIO0 -> FIODIR &= ~ (1<<1); // PO.1
      // LPC_GPIO0 -> FIODIR &= ~ (1<<0); // P0.0
/*
         LPC_GPIOINT->IO0IntEnR |=(1<<1);   // Enabling Rising Interrupt for p1.1
        //LPC_GPIOINT->IO0IntEnF |=(1<<1);
        LPC_GPIOINT->IO0IntEnR |=(1<<0);     // Enabling Rising Interrupt for p1.0
       // LPC_GPIOINT->IO0IntEnF |=(1<<0);
        LPC_GPIOINT->IO0IntClr = 0xFFFFFFFF;  // Clearing all the interupts for p0

        NVIC_EnableIRQ(EINT3_IRQn);             // Port 0 and 2 needs 21 interupt number
 */
 //   gQueue=xQueueCreate(1,sizeof(int));
       // xTaskCreate(motor_task1,"first_task",4096,NULL,2,NULL);
    eint3_enable_port0(1,eint_falling_edge, call);

    xTaskCreate(sensor_task1,"Second_task",4096,NULL,1,NULL);
        vTaskStartScheduler();

        //  xTaskCreate(motor_task1, "motor_task", 2048, NULL, 1, NULL);
           //xTaskCreate(sensor_task1, "sensor_task", 5000, NULL, 1, NULL);
    //can_world_init();
    //LS.init();
    /**
     * A few basic tasks for this bare-bone system :
     *      1.  Terminal task provides gateway to interact with the board through UART terminal.
     *      2.  Remote task allows you to use remote control to interact with the board.
     *      3.  Wireless task responsible to receive, retry, and handle mesh network.
     *
     * Disable remote task if you are not using it.  Also, it needs SYS_CFG_ENABLE_TLM
     * such that it can save remote control codes to non-volatile memory.  IR remote
     * control codes can be learned by typing the "learn" terminal command.
     */
    //scheduler_add_task(new terminalTask(PRIORITY_HIGH));
    //scheduler_add_task(new Servomotor_pwm(PRIORITY_HIGH));
    /* Consumes very little CPU, but need highest priority to handle mesh network ACKs */
    //scheduler_add_task(new wirelessTask(PRIORITY_CRITICAL));

    /* Change "#if 0" to "#if 1" to run period tasks; @see period_callbacks.cpp */
    #if 0
    scheduler_add_task(new periodicSchedulerTask());
    #endif

    /* The task for the IR receiver */
    // scheduler_add_task(new remoteTask  (PRIORITY_LOW));

    /* Your tasks should probably used PRIORITY_MEDIUM or PRIORITY_LOW because you want the terminal
     * task to always be responsive so you can poke around in case something goes wrong.
     */

    /**
     * This is a the board demonstration task that can be used to test the board.
     * This also shows you how to send a wireless packets to other boards.
     */
    #if 0
        scheduler_add_task(new example_io_demo());
    #endif

    /**
     * Change "#if 0" to "#if 1" to enable examples.
     * Try these examples one at a time.
     */
    #if 0
        scheduler_add_task(new example_task());
        scheduler_add_task(new example_alarm());
        scheduler_add_task(new example_logger_qset());
        scheduler_add_task(new example_nv_vars());
    #endif

    /**
     * Try the rx / tx tasks together to see how they queue data to each other.
     */
    #if 0
        scheduler_add_task(new queue_tx());
        scheduler_add_task(new queue_rx());
    #endif

    /**
     * Another example of shared handles and producer/consumer using a queue.
     * In this example, producer will produce as fast as the consumer can consume.
     */
    #if 0
        scheduler_add_task(new producer());
        scheduler_add_task(new consumer());
    #endif

    /**
     * If you have RN-XV on your board, you can connect to Wifi using this task.
     * This does two things for us:
     *   1.  The task allows us to perform HTTP web requests (@see wifiTask)
     *   2.  Terminal task can accept commands from TCP/IP through Wifly module.
     *
     * To add terminal command channel, add this at terminal.cpp :: taskEntry() function:
     * @code
     *     // Assuming Wifly is on Uart3
     *     addCommandChannel(Uart3::getInstance(), false);
     * @endcode
     */
    #if 0
        Uart3 &u3 = Uart3::getInstance();
        u3.init(WIFI_BAUD_RATE, WIFI_RXQ_SIZE, WIFI_TXQ_SIZE);
        scheduler_add_task(new wifiTask(Uart3::getInstance(), PRIORITY_LOW));
    #endif

    scheduler_start(); ///< This shouldn't return
    return 0;
}


































#if 0

/* *     SocialLedge.com - Copyright (C) 2013
 *
 *     This file is part of free software framework for embedded processors.
 *     You can use it and/or distribute it as long as this copyright header
 *     remains unmodified.  The code is free for personal use and requires
 *     permission to use in a commercial product.
 *
 *      THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 *      OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 *      MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 *      I SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 *      CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *     You can reach the author of this software at :
 *          p r e e t . w i k i @ g m a i l . c o m
 */

/**
 * @file
 * @brief This is the application entry point.
 * 			FreeRTOS and stdio printf is pre-configured to use uart0_min.h before main() enters.
 * 			@see L0_LowLevel/lpc_sys.h if you wish to override printf/scanf functions.
 *
 */



#include "tasks.hpp"
#include "examples/examples.hpp"
#include "file_logger.h"

/**
 * The main() creates tasks or "threads".  See the documentation of scheduler_task class at scheduler_task.hpp
 * for details.  There is a very simple example towards the beginning of this class's declaration.
 *
 * @warning SPI #1 bus usage notes (interfaced to SD & Flash):
 *      - You can read/write files from multiple tasks because it automatically goes through SPI semaphore.
 *      - If you are going to use the SPI Bus in a FreeRTOS task, you need to use the API at L4_IO/fat/spi_sem.h
 *
 * @warning SPI #0 usage notes (Nordic wireless)
 *      - This bus is more tricky to use because if FreeRTOS is not running, the RIT interrupt may use the bus.
 *      - If FreeRTOS is running, then wireless task may use it.
 *        In either case, you should avoid using this bus or interfacing to external components because
 *        there is no semaphore configured for this bus and it should be used exclusively by nordic wireless.
 */


#include "io.hpp"
#include "utilities.h"
#include "can.h"
#include "stdio.h"
#include "LED.hpp"
#include "switches.hpp"

#include "gpio.hpp"

 /*
 *  The main() creates tasks or "threads".  See the documentation of scheduler_task class at scheduler_task.hpp
 * for details.  There is a very simple example towards the beginning of this class's declaration.
 *
 * @warning SPI #1 bus usage notes (interfaced to SD & Flash):
 *      - You can read/write files from multiple tasks because it automatically goes through SPI semaphore.
 *      - If you are going to use the SPI Bus in a FreeRTOS task, you need to use the API at L4_IO/fat/spi_sem.h
 *
 * @warning SPI #0 usage notes (Nordic wireless)
 *      - This bus is more tricky to use because if FreeRTOS is not running, the RIT interrupt may use the bus.
 *      - If FreeRTOS is running, then wireless task may use it.
 *        In either case, you should avoid using this bus or interfacing to external components because
 *        there is no semaphore configured for this bus and it should be used exclusively by nordic wireless.
 */


// Task for receiver.
void CAN_read_task (void)
{
    while(1)
    {
        can_msg_t rxmsg;
        int id;
        rxmsg.data.qword = 0;
        if(CAN_rx(can1, &rxmsg, 100))
        {
            id=rxmsg.msg_id;
           // printf("Id  : %d\n", rxmsg.msg_id);

            //0x100 corresponds to 256 in decimal. So if receiver task get msgId 100, it will turn On the LED
            if (id==256)
            {
                printf("id is %d and in hex %x\n",id,rxmsg.msg_id);
                LE.on(1);
            }
            //0x200 corresponds to 512 in decimal. Hence, if receiver task gets messageId 512, it will turn off the LED.
            if (id==512)
               {
                printf("id is %d and in hex %x\n",id,rxmsg.msg_id);
                LE.off(1);
                }
/*
            printf("Data: %c", rxmsg.data.bytes[0]);
            printf(" %x", rxmsg.data.bytes[1]);
            printf(" %x", rxmsg.data.bytes[2]);
            printf(" %x", rxmsg.data.bytes[3]);
            printf(" %x", rxmsg.data.bytes[4]);
            printf(" %x", rxmsg.data.bytes[5]);
            printf(" %x", rxmsg.data.bytes[6]);
            printf(" %x\n", rxmsg.data.bytes[7]);
 */

            }
        }
    }


// Task for Transmitter
class can2Task : public scheduler_task
{
   public:
       can2Task(uint8_t priority) :
           scheduler_task("can", 2048, priority)
       {
           printf("inside init\n");
           CAN_init(can1, 100, 10, 10, NULL, NULL);
           CAN_reset_bus(can1);
           SW.init();
       }

       bool run(void *p)
       {
           can_msg_t txmsg;
           txmsg.frame_fields.is_29bit = 0;
           txmsg.frame_fields.data_len = 8;

           //If switch 1 is kept pressed, then SW.getSwitch(1) will retunr true.
           bool check_switch=SW.getSwitch(1);
           if (check_switch == true)
           {
               txmsg.msg_id = 0x100;
           }
           //On releasing the switch 1, check_switch will be false.
               else
               {
               txmsg.msg_id = 0x200;
               }
           txmsg.data.bytes[8]=0x1122334455667788;
           if(CAN_tx(can1, &txmsg, portMAX_DELAY))
               {
                   printf("Sent message id is %x \n",txmsg.msg_id);
                }

           delay_ms(500);
           return true;
       }
};


int main(void)
{

    scheduler_add_task(new terminalTask(PRIORITY_HIGH));
#ifndef RX
      scheduler_add_task(new can2Task(PRIORITY_HIGH));
#else
     CAN_init(can1, 100, 10, 10, NULL, NULL);
     //CAN_bypass_filter_accept_all_msgs();

    const can_std_id_t slist[]      = { CAN_gen_sid(can1, 0x100),  CAN_gen_sid(can1, 0x200)};
    const can_std_grp_id_t sglist[] = { {CAN_gen_sid(can1, 0x100), CAN_gen_sid(can1, 0x2FF)}       };

        const can_ext_id_t *elist       = NULL;
       const can_ext_grp_id_t eglist[] = { {CAN_gen_eid(can1, 0x4500), CAN_gen_eid(can1, 0x5500)} };

       CAN_setup_filter(slist, 2, sglist, 1, elist, 0, eglist, 1);

        CAN_reset_bus(can1);
        LE.init();   // Initializing the LED.
    xTaskCreate((void(*)(void *))CAN_read_task, "Receive_task", 1024, NULL, PRIORITY_HIGH, NULL);
#endif

    /* Consumes very little CPU, but need highest priority to handle mesh network ACKs */
  //  scheduler_add_task(new wirelessTask(PRIORITY_CRITICAL));
  //  scheduler_add_task(new canTx(PRIORITY_HIGH));
    /* Change "#if 0" to "#if 1" to run period tasks; @see period_callbacks.cpp */
    #if 0
    scheduler_add_task(new periodicSchedulerTask());
    #endif

    /* The task for the IR receiver */
    // scheduler_add_task(new remoteTask  (PRIORITY_LOW));

    /* Your tasks should probably used PRIORITY_MEDIUM or PRIORITY_LOW because you want the terminal
     * task to always be responsive so you can poke around in case something goes wrong.
     */

    /**
     * This is a the board demonstration task that can be used to test the board.
     * This also shows you how to send a wireless packets to other boards.
     */
    #if 0
        scheduler_add_task(new example_io_demo());
    #endif

    /**
     * Change "#if 0" to "#if 1" to enable examples.
     * Try these examples one at a time.
     */
    #if 0
        scheduler_add_task(new example_task());
        scheduler_add_task(new example_alarm());
        scheduler_add_task(new example_logger_qset());
        scheduler_add_task(new example_nv_vars());
    #endif

    /**
	 * Try the rx / tx tasks together to see how they queue data to each other.
	 */
    #if 0
        scheduler_add_task(new queue_tx());
        scheduler_add_task(new queue_rx());
    #endif

    /**
     * Another example of shared handles and producer/consumer using a queue.
     * In this example, producer will produce as fast as the consumer can consume.
     */
    #if 0
        scheduler_add_task(new producer());
        scheduler_add_task(new consumer());
    #endif

    /**
     * If you have RN-XV on your board, you can connect to Wifi using this task.
     * This does two things for us:
     *   1.  The task allows us to perform HTTP web requests (@see wifiTask)
     *   2.  Terminal task can accept commands from TCP/IP through Wifly module.
     *
     * To add terminal command channel, add this at terminal.cpp :: taskEntry() function:
     * @code
     *     // Assuming Wifly is on Uart3
     *     addCommandChannel(Uart3::getInstance(), false);
     * @endcode
     */
    #if 0
        Uart3 &u3 = Uart3::getInstance();
        u3.init(WIFI_BAUD_RATE, WIFI_RXQ_SIZE, WIFI_TXQ_SIZE);
        scheduler_add_task(new wifiTask(Uart3::getInstance(), PRIORITY_LOW));
    #endif

    scheduler_start(); ///< This shouldn't return
    return -1;
}
#endif
