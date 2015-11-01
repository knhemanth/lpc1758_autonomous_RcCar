/*
 *     SocialLedge.com - Copyright (C) 2013
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
 * This contains the period callback functions for the periodic scheduler
 *
 * @warning
 * These callbacks should be used for hard real-time system, and the priority of these
 * tasks are above everything else in the system (above the PRIORITY_CRITICAL).
 * The period functions SHOULD NEVER block and SHOULD NEVER run over their time slot.
 * For example, the 1000Hz take slot runs periodically every 1ms, and whatever you
 * do must be completed within 1ms.  Running over the time slot will reset the system.
 */
#include <stdint.h>
#include<ir.h>
#include "io.hpp"
#include "periodic_callback.h"
#include "ultrasonic_sensor.hpp"
#include "queue.h"

/// This is the stack size used for each of the period tasks
const uint32_t PERIOD_TASKS_STACK_SIZE_BYTES = (512 * 4);

// This enumeration matches the distance of the obstacle
typedef enum {
    nearest = 0,
    near    = 1,
    middle  = 2,
    far     = 3,
} distance_obstacle;

char zoneMessage[4][10] = {"nearest", "near", "middle", "far"};

//This enumeration matches the threshold of the obstacle
typedef enum{
    threshold_zero    = 0 ,
    threshold_nearest = 50,
    threshold_near    = 100,
    threshold_middle  = 200,
    thrshold_far      = 400,
} obs_thre;

//Divya editing master module for ultrasonic sensor
//Creating a class instance of all the ultrasonic modules
Ultra_Sonic_4ping front_ultrasonic(P2_0, P2_1);


void period_1Hz(void)
{
    LE.toggle(1);
}

void period_10Hz(void)
{

    LE.toggle(2);
}

void period_100Hz(void)
{

      float latest_value = 0;
      float avg_distance_of_obstacle = 0;
      distance_obstacle obstacle_zone;
      obs_thre threshold;


    LE.toggle(3);

    //Ping Sensor input
     front_ultrasonic.send_trig();

     if(front_ultrasonic.recieve_from_queue())  //if the queue has a value->returns true
     {
         latest_value = front_ultrasonic.get_buffer_value();


         front_ultrasonic.add_queue_value_to_filter(latest_value);  //adding a float values in the filter

         avg_distance_of_obstacle = front_ultrasonic.ping_get_from_filter(); //returns the latest average value of the distances


         //printf("%f: \n",avg_distance_of_obstacle);
     }
     else
     {
         //printf("No values in the filter\n");
     }

    // front_ultrasonic.display();
#if 1
            if( (avg_distance_of_obstacle >= threshold_zero) && (avg_distance_of_obstacle <= threshold_nearest))
            {
                obstacle_zone = nearest;
            }
            else if((avg_distance_of_obstacle > threshold_nearest) && (avg_distance_of_obstacle <= threshold_near))
            {
                   obstacle_zone = near;
            }
            else if((avg_distance_of_obstacle > threshold_near) && (avg_distance_of_obstacle <= threshold_middle))
                     {
                            obstacle_zone = middle;
                     }
                else if((avg_distance_of_obstacle > threshold_middle) && (avg_distance_of_obstacle <= thrshold_far))
                     {
                            obstacle_zone = far;
                     }

            printf("Z[%s]\n", zoneMessage[obstacle_zone]);
#endif
}


void period_1000Hz(void)
{
    LE.toggle(4);

}
