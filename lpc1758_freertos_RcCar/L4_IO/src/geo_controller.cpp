/*
 * geo_controller.cpp
 *
 *  Created on: Oct 25, 2015
 *      Author: Hemanth K N
 */

#include <string.h>
#include "geo_controller.hpp"
#include "can_msg_id.h"
#include "io.hpp"
#include "can.h"
#include "can_msg_id.h"
#include "soft_timer.hpp"
#include "imu.hpp"



static bool power_up_sync_geo_controller( void );
static can_std_id_t can_id_sync_ack;       // Ack from master
static float gps_lat = 0;
static float gps_long = 0;
static float gps_speed = 0;

bool geo_controller_init( void )
{
    can_std_id_t can_id_loc_update;     // Location update from master
    bool status = false;

    can_id_sync_ack = CAN_gen_sid(GEO_CNTL_CANBUS, MASTER_SYNC_ACK_ID);
    can_id_loc_update = CAN_gen_sid(GEO_CNTL_CANBUS, GEO_LOC_UPDATE_ID);


    // Initialize the can bus
    status = CAN_init(GEO_CNTL_CANBUS, GEO_CNTL_BAUD, GEO_CNTL_CANRXQ, GEO_CNTL_CANTXQ , bus_off_cb, data_ovr_cb);
    if( !status )
    {
        LOG_ERROR("ERROR!!! Cannot initialize GEO controller CAN Bus!!");
        return status;
    }

    // Setup full can filters
    status = CAN_fullcan_add_entry(GEO_CNTL_CANBUS, can_id_sync_ack, can_id_loc_update);
    if( !status )
    {
        LOG_ERROR("ERROR!!! Cannot add FullCAN entries to GEO controller CAN Bus!!");
        return status;
    }

    // Enable the bus
    CAN_reset_bus(GEO_CNTL_CANBUS);

    // Sync with the master controller by sending power_up_sync
    status = power_up_sync_geo_controller();

    return status;
}

void bus_off_cb( uint32_t icr_data )
{
    // XXX: This callback occurs from inside an ISR, so cannot log or print anything
    // Also, bus reset should happen at maybe 1 or 10Hz, but not immediately
    //LOG_ERROR("ERROR!!! GEO Controller CAN bus in off state. Resetting bus");
    CAN_reset_bus(GEO_CNTL_CANBUS);
}

void data_ovr_cb( uint32_t icr_data )
{
    //LOG_WARN("WARNING!!! GEO Controller CAN bus overflow!!!");
    CAN_reset_bus(GEO_CNTL_CANBUS);
}

bool power_up_sync_geo_controller( void )
{
    can_msg_t geo_sync_msg;
    can_fullcan_msg_t *geo_syncack_msg_ptr = NULL;
    can_fullcan_msg_t geo_syncack_msg_copy;
    uint8_t sync_miss_count = 0;
    bool sync_ack = false;
    bool status = false;
    master_sync* sync_msg;
    SoftTimer can_rx_timer(GEO_CNTL_SYNC_TIME);

    geo_syncack_msg_ptr = CAN_fullcan_get_entry_ptr(can_id_sync_ack);
    if( geo_syncack_msg_ptr == NULL )
    {
        LOG_ERROR("ERROR!!! Cannot get FullCan msg pointer for Geo controller power-up sync\n");
        return false;
    }

    geo_sync_msg.msg_id = GEO_SYNC_ID;
    geo_sync_msg.frame_fields.is_29bit = 0;
    geo_sync_msg.frame_fields.data_len = 0;     // No data
    geo_sync_msg.data.qword = 0;


    do
    {
        // Send sync message
        status = CAN_tx(GEO_CNTL_CANBUS, &geo_sync_msg, GEO_CNTL_CAN_TIMEOUT);
        if( !status )
            LOG_ERROR("ERROR!!! Unable to send Geo controller sync message\n");

        // No need to delay here
        // XXX: Cannot use FreeRTOS functions until the OS runs
        // vTaskDelayMs(GEO_CNTL_SYNC_TIME);


        can_rx_timer.restart();
        while( !can_rx_timer.expired() );


        // Check for ack
        status = CAN_fullcan_read_msg_copy( geo_syncack_msg_ptr, &geo_syncack_msg_copy );

        // XXX: Since OS is not up, maybe you can use SoftTime (polling timer)
        if( status )
        {
            // We have a new message. Check if Geo is acked

            // XXX: Suggest "shared" structures rather than memcpy
            //memcpy((void *)&sync_msg, (void *)&geo_syncack_msg_copy.data.qword, sizeof(sync_msg));
            sync_msg = (master_sync *)&geo_syncack_msg_copy.data.bytes[0];
            if( sync_msg->ack_geo )
                sync_ack = true;

        }

        sync_miss_count++;

    }while( ( sync_ack == false ) );    // try forever

#if 0
    if( !sync_ack )
        LOG_ERROR("ERROR!!! Sync timeout from Geo controller. Stalling!!!\n");
#endif

    return sync_ack;
}

#ifdef __cplusplus
extern "C"{
#endif

void geo_send_gps( void )
{
    can_msg_t gps_msg;
    geo_loc gps_data;
    bool can_status = false;

    gps_data.latitude = gps_lat;
    gps_data.longitude = gps_long;

    gps_msg.msg_id = GEO_LOC_DATA_ID;
    gps_msg.frame_fields.is_29bit = 0;
    gps_msg.frame_fields.data_len = sizeof(gps_data);
    memcpy(&gps_msg.data.qword, &gps_msg, sizeof(gps_msg));

    can_status = CAN_tx(GEO_CNTL_CANBUS, &gps_msg, GEO_CNTL_CAN_TIMEOUT);

    if( !can_status )
        {
            LOG_ERROR("ERROR!!! Geo controller CAN message: GPS data not sent!!");
            LE.on(1);
        }

    else
        LE.off(1);

}

void geo_send_heading( void )
{
    static imu& imu_handle = IMUInterface;  // Handle to singleton IMU object
    uint16_t imu_heading = 0;       // 2-byte angle between 0 and 360. Compromise with precision
    geo_spd_angle geo_data;
    can_msg_t geo_msg;
    bool can_status = false;

    imu_heading = static_cast<uint16_t>(imu_handle.getHeading());

    // Get gps_lat, gps_long and gps_speed from GPS sensors
    gps_lat = 0;
    gps_long = 0;
    gps_speed = 0;

    // Call func to calculate bearing

    geo_data.bearing = 0;   // put bearing here
    geo_data.heading = imu_heading;
    geo_data.speed = gps_speed;     // Put speed from GPS here

    geo_msg.msg_id = GEO_SPEED_ANGLE_ID;
    geo_msg.frame_fields.is_29bit = 0;
    geo_msg.frame_fields.data_len = sizeof(geo_data);
    memcpy(&geo_msg.data.qword,&geo_data, sizeof(geo_data));


    // CAN_Tx() will only time out when TX queue is full and that will only
    // happen when CAN Bus turns off too long for us to empty the TX queue
    can_status = CAN_tx(GEO_CNTL_CANBUS, &geo_msg, GEO_CNTL_CAN_TIMEOUT);

    if( !can_status )
    {
        LOG_ERROR("ERROR!!! Geo controller CAN message: IMU data not sent!!");
        LE.on(1);
    }

    else
        LE.off(1);
}

void geo_send_heartbeat( void )
{
    // Heart Beat to Master
    can_msg_t heartbeat_geo_msg; // Can Message
    bool can_status = false;

    heartbeat_geo_msg.msg_id = GEO_HEARTBEAT_ID; // Geo Heartbeat ID
    heartbeat_geo_msg.frame_fields.is_29bit = 0;
    heartbeat_geo_msg.frame_fields.data_len = 0;

    can_status = CAN_tx(GEO_CNTL_CANBUS, &heartbeat_geo_msg, GEO_CNTL_CAN_TIMEOUT);

    if( !can_status )
    {
        LOG_ERROR("ERROR!!! Geo controller Heartbeat message not sent!!");
        LE.off(GEO_HB_LED);
    }
    else
    {
        LE.toggle(GEO_HB_LED);
    }
}

#ifdef __cplusplus
}
#endif
