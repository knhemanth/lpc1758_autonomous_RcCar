/*
 * master_controller.cpp
 *
 *  Created on: Oct 29, 2015
 *      Author: Hemanth K N
 */

#include "master_controller.hpp"
#include "io.hpp"
#include "can_msg_id.h"
#include "can.h"
#include "file_logger.h"


/* Global ID variables - Should be used as read only */
can_std_id_t can_id_kill;
can_std_id_t can_id_motorio;
can_std_id_t can_id_sensor;
can_std_id_t can_id_bluetooth;
can_std_id_t can_id_geo;
can_std_id_t can_id_motor_hb;
can_std_id_t can_id_sensor_hb;
can_std_id_t can_id_bluetooth_hb;
can_std_id_t can_id_geo_hb;
can_std_id_t can_id_runmode;
can_std_id_t can_id_distance;
can_std_id_t can_id_chkpt_snd;
can_std_id_t can_id_chkpt_data;
can_std_id_t can_id_spd_angle;
can_std_id_t can_id_loc_data;

/* Static function prototypes */
void power_up_sync_and_ack( void );

bool master_controller_init()
{

    /*
     *  Master needs to read the following CAN messages:

        KILL_SWITCH_ID              (0x00)

        MOTORIO_SYNC_ID             (0x03)
        SENSOR_SYNC_ID              (0x04)
        BLUETOOTH_SYNC_ID           (0x05)
        GEO_SYNC_ID                 (0x06)

        MOTORIO_HEARTBEAT_ID        (0x07)
        SENSOR_HEARTBEAT_ID         (0x08)
        BLUETOOTH_HEARTBEAT_ID      (0x09)
        GEO_HEARTBEAT_ID            (0x0A)

        RUN_MODE_ID                 (0x0B)

        DISTANCE_SENSOR_ID          (0x0C)

        CHECKPOINT_SEND_ID          (0x0F)
        CHECKPOINT_DATA_ID          (0x10)

        GEO_SPEED_ANGLE_ID          (0x12)
        GEO_LOC_DATA_ID             (0x13)

     */

    bool status = false;
    can_std_id_t dummy;

    // Setup the CAN bus for the master controller
    status = CAN_init(MASTER_CNTL_CANBUS, MASTER_CNTL_BAUD, MASTER_CNTL_RXQ, MASTER_CNTL_TXQ, bus_off_cb, data_ovr_cb);

    // Generate IDs
    can_id_kill = CAN_gen_sid(MASTER_CNTL_CANBUS, KILL_SWITCH_ID);
    can_id_motorio = CAN_gen_sid(MASTER_CNTL_CANBUS, MOTORIO_SYNC_ID);;
    can_id_sensor = CAN_gen_sid(MASTER_CNTL_CANBUS, SENSOR_SYNC_ID);;
    can_id_bluetooth = CAN_gen_sid(MASTER_CNTL_CANBUS, BLUETOOTH_SYNC_ID);;
    can_id_geo = CAN_gen_sid(MASTER_CNTL_CANBUS, GEO_SYNC_ID);;
    can_id_motor_hb = CAN_gen_sid(MASTER_CNTL_CANBUS, MOTORIO_HEARTBEAT_ID);;
    can_id_sensor_hb = CAN_gen_sid(MASTER_CNTL_CANBUS, SENSOR_HEARTBEAT_ID);;
    can_id_bluetooth_hb = CAN_gen_sid(MASTER_CNTL_CANBUS, BLUETOOTH_HEARTBEAT_ID);;
    can_id_geo_hb = CAN_gen_sid(MASTER_CNTL_CANBUS, GEO_HEARTBEAT_ID);;
    can_id_runmode = CAN_gen_sid(MASTER_CNTL_CANBUS, RUN_MODE_ID);;
    can_id_distance = CAN_gen_sid(MASTER_CNTL_CANBUS, DISTANCE_SENSOR_ID);;
    can_id_chkpt_snd = CAN_gen_sid(MASTER_CNTL_CANBUS, CHECKPOINT_SEND_ID);;
    can_id_chkpt_data = CAN_gen_sid(MASTER_CNTL_CANBUS, CHECKPOINT_DATA_ID);;
    can_id_spd_angle = CAN_gen_sid(MASTER_CNTL_CANBUS, GEO_SPEED_ANGLE_ID);;
    can_id_loc_data = CAN_gen_sid(MASTER_CNTL_CANBUS, GEO_LOC_DATA_ID);;
    dummy = CAN_gen_sid(MASTER_CNTL_CANBUS, DUMMY_ID );

    // Add fullCAN entries
    status = CAN_fullcan_add_entry(MASTER_CNTL_CANBUS, can_id_kill, can_id_motorio);
    status = CAN_fullcan_add_entry(MASTER_CNTL_CANBUS, can_id_sensor, can_id_bluetooth);
    status = CAN_fullcan_add_entry(MASTER_CNTL_CANBUS, can_id_geo, can_id_motor_hb);
    status = CAN_fullcan_add_entry(MASTER_CNTL_CANBUS, can_id_sensor_hb, can_id_bluetooth_hb);
    status = CAN_fullcan_add_entry(MASTER_CNTL_CANBUS, can_id_geo_hb, can_id_runmode);
    status = CAN_fullcan_add_entry(MASTER_CNTL_CANBUS, can_id_distance, can_id_chkpt_snd);
    status = CAN_fullcan_add_entry(MASTER_CNTL_CANBUS, can_id_chkpt_data, can_id_spd_angle);
    status = CAN_fullcan_add_entry(MASTER_CNTL_CANBUS, can_id_loc_data, dummy);

    if( !status )
    {
        LOG_ERROR("ERROR!!! Master controller - Cannot setup fullCAN entries\n");
    }

    //reset the can bus to enable it now
    CAN_reset_bus(MASTER_CNTL_CANBUS);

    power_up_sync_and_ack();

    return status;
}

void power_up_sync_and_ack( void )
{



}
