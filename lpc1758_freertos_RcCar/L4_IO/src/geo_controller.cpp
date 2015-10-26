/*
 * geo_controller.cpp
 *
 *  Created on: Oct 25, 2015
 *      Author: Hemanth K N
 */

#include "geo_controller.hpp"
#include "can_msg_id.h"
#include "io.hpp"
#include "can.h"

bool geo_controller_init( void )
{
    can_std_id_t can_id_sync_ack;       // Ack from master
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

    return status;
}

void bus_off_cb( uint32_t icr_data )
{
    LOG_ERROR("ERROR!!! GEO Controller CAN bus in off state. Resetting bus");

    CAN_reset_bus(GEO_CNTL_CANBUS);
}

void data_ovr_cb( uint32_t icr_data )
{
    LOG_WARN("WARNING!!! GEO Controller CAN bus overflow!!!");
}

