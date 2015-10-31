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
#include "io.hpp"
#include "periodic_callback.h"
#include "geo_controller.hpp"
#include "can_msg_id.h"
#include "can.h"

#include <stdio.h>


/// This is the stack size used for each of the period tasks
const uint32_t PERIOD_TASKS_STACK_SIZE_BYTES = (512 * 4);

bool hb_motor = false;
bool hb_geo = false;
bool hb_sensor = false;
bool hb_bluetooth = false;

bool sync_motor = false;
bool sync_geo = false;
bool sync_sensor = false;
bool sync_bluetooth = false;

void send_can_msg(uint32_t id)
{
    can_msg_t msg;

    msg.msg_id = id;
    msg.frame_fields.is_29bit = 0;
    msg.frame_fields.data_len = 0;
    msg.data.qword = 0;

    CAN_tx( can2, &msg, 0);

}

void period_1Hz(void)
{
    // Power Up Sync Simulation
    if(sync_motor)
    {
       send_can_msg(MOTORIO_SYNC_ID);
       printf("Power Up Sync : Motor Sent!\n");

    }

    if(sync_geo)
    {
       send_can_msg(GEO_SYNC_ID);
       printf("Power Up Sync  : Geo Sent!\n");

    }

    if(sync_sensor)
    {
       send_can_msg(SENSOR_SYNC_ID);
       printf("Power Up Sync  : Sensor Sent!\n");

    }

    if(sync_bluetooth)
    {
       send_can_msg(BLUETOOTH_SYNC_ID);
       printf("Power Up Sync  : Bluetooth Sent!\n");

    }

       // Heartbeat Simulation
    if(hb_motor)
    {
        send_can_msg(MOTORIO_HEARTBEAT_ID);
        printf("Heart Beat : Motor Sent!\n");

    }

    if(hb_geo)
    {
        send_can_msg(GEO_HEARTBEAT_ID);
        printf("Heart Beat : Geo Sent!\n");

    }

    if(hb_sensor)
    {
        send_can_msg(SENSOR_HEARTBEAT_ID);
        printf("Heart Beat : Sensor Sent!\n");

    }

    if(hb_bluetooth)
    {
        send_can_msg(BLUETOOTH_HEARTBEAT_ID);
        printf("Heart Beat : Bluetooth Sent!\n");

    }

}

void period_10Hz(void)
{


}

void period_100Hz(void)
{

}

void period_1000Hz(void)
{

}
