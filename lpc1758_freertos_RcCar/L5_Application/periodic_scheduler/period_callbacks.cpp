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


#include <stdio.h>



/// This is the stack size used for each of the period tasks
const uint32_t PERIOD_TASKS_STACK_SIZE_BYTES = (512 * 4);


void period_1Hz(void)
{
    // Heart Beat to Master
    static heart_beat heartbeat_message; // 1 Byte Data
    can_msg_t heartbeat_geo_msg; // Can Message
    bool can_status = false;

    heartbeat_message.counter++; // Increment Heartbeat Count

    heartbeat_geo_msg.msg_id = GEO_HEARTBEAT_ID; // Geo Heartbeat ID
    heartbeat_geo_msg.frame_fields.is_29bit = 0;
    heartbeat_geo_msg.frame_fields.data_len = sizeof(heart_beat);

    memcpy((void *)&heartbeat_geo_msg.data.qword, (void *)&heartbeat_message, sizeof(heartbeat_message)); // 1 Byte Data

    can_status = CAN_tx(GEO_CNTL_CANBUS, &heartbeat_geo_msg, GEO_CNTL_CAN_TIMEOUT);

    if( !can_status )
    {
        LOG_ERROR("ERROR!!! Geo controller Heartbeat message not sent!!");
        LE.off(4);
    }
    else
    {
        LE.toggle(4);
    }
}

void period_10Hz(void)
{

    /*
     * TODO: For GPS team
     * Don't change the order of these calls.
     * If you change then call gps getters in both functions
     */
    geo_send_heading();
    geo_send_gps();
}

void period_100Hz(void)
{
    /* Nothing will be done here */
}

void period_1000Hz(void)
{
    /*Nothing will be done here */
}
