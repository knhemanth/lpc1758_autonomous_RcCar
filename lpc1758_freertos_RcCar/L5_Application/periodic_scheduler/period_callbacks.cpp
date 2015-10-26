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
#include "imu.hpp"
#include "can_msg_id.h"
#include "can.h"

#include <stdio.h>



/// This is the stack size used for each of the period tasks
const uint32_t PERIOD_TASKS_STACK_SIZE_BYTES = (512 * 4);



void period_1Hz(void)
{

}

void period_10Hz(void)
{
    static imu& imu_handle = IMUInterface;  // Handle to singleton IMU object
        uint16_t imu_heading = 0;       // 2-byte angle between 0 and 360. Compromise with precision
        float gps_lat = 0;
        float gps_long = 0;
        geo_spd_angle geo_data;
        geo_loc gps_data;
        can_msg_t geo_msg;
        can_msg_t gps_msg;
        bool can_status = false;

        imu_heading = static_cast<uint16_t>(imu_handle.getHeading());

        // Get gps_lat and gps_long from GPS sensors

        // Call func to calculate bearing

        geo_data.bearing = 0;   // put bearing here
        geo_data.heading = imu_heading;
        geo_data.speed = 0;     // Put speed from GPS here

        gps_data.latitude = gps_lat;
        gps_data.longitude = gps_long;


        geo_msg.msg_id = GEO_SPEED_ANGLE_ID;
        geo_msg.frame_fields.is_29bit = 0;
        geo_msg.frame_fields.data_len = sizeof(geo_data);
        memcpy((void *)&geo_msg.data.qword, (void *)&geo_data, sizeof(geo_data));

        gps_msg.msg_id = GEO_LOC_DATA_ID;
        gps_msg.frame_fields.is_29bit = 0;
        gps_msg.frame_fields.data_len = sizeof(gps_data);
        memcpy((void *)&gps_msg.data.qword, (void *)&gps_data, sizeof(gps_data));

        can_status = CAN_tx(GEO_CNTL_CANBUS, &geo_msg, GEO_CNTL_CAN_TIMEOUT);
        can_status = CAN_tx(GEO_CNTL_CANBUS, &gps_msg, GEO_CNTL_CAN_TIMEOUT);

        if( !can_status )
        {
            LOG_ERROR("ERROR!!! Geo controller CAN message not sent!!");
            LE.on(1);
        }

        else
            LE.off(1);

}

void period_100Hz(void)
{

}

void period_1000Hz(void)
{

}
