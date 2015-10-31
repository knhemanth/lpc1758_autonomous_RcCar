/*
 * master_controller.cpp
 *
 *  Created on: Oct 29, 2015
 *      Author: Hemanth K N
 */

#include <string.h>
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

/* Static variables */
static bool motorio_sync = false;
static bool bluetooth_sync = false;
static bool geo_sync = false;
static bool sensor_sync = false;

/* Static function prototypes */
static bool power_up_sync_and_ack( void );
static void bus_off_cb( uint32_t ICR_data );
static void data_ovr_cb( uint32_t ICR_data );


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

    while( !( power_up_sync_and_ack() ) );

    return status;
}

static bool power_up_sync_and_ack( void )
{
    can_fullcan_msg_t* can_msg_motorio_sync_ptr = NULL;
    can_fullcan_msg_t* can_msg_sensor_sync_ptr = NULL;
    can_fullcan_msg_t* can_msg_bluetooth_sync_ptr = NULL;
    can_fullcan_msg_t* can_msg_geo_sync_ptr = NULL;

    can_msg_motorio_sync_ptr = CAN_fullcan_get_entry_ptr(can_id_motorio);
    can_msg_sensor_sync_ptr = CAN_fullcan_get_entry_ptr(can_id_sensor);
    can_msg_bluetooth_sync_ptr = CAN_fullcan_get_entry_ptr(can_id_bluetooth);
    can_msg_geo_sync_ptr = CAN_fullcan_get_entry_ptr(can_id_geo);

    //sync now
    master_sync master_ack_msg;
    master_ack_msg.ack_bluetooth = (bluetooth_sync)?ACK:NACK;
    master_ack_msg.ack_geo = (geo_sync)?ACK:NACK;
    master_ack_msg.ack_motorio = (motorio_sync)?ACK:NACK;
    master_ack_msg.ack_sensor = (sensor_sync)?ACK:NACK;

    bool synced = false;
    bool status = false;


    can_fullcan_msg_t controller_sync_msg;

    if( !motorio_sync ){
        status = CAN_fullcan_read_msg_copy(can_msg_motorio_sync_ptr, &controller_sync_msg);
        if(status) {
            motorio_sync = true;
            master_ack_msg.ack_motorio = ACK;
        }
    }

    if( !bluetooth_sync ){
        status = CAN_fullcan_read_msg_copy(can_msg_bluetooth_sync_ptr, &controller_sync_msg);
        if(status) {
            bluetooth_sync = true;
            master_ack_msg.ack_bluetooth = ACK;
        }
    }

    if( !sensor_sync ) {
        status = CAN_fullcan_read_msg_copy(can_msg_sensor_sync_ptr, &controller_sync_msg);
        if(status) {
            sensor_sync = true;
            master_ack_msg.ack_sensor = ACK;
        }
    }


    if( !geo_sync ) {
        status = CAN_fullcan_read_msg_copy(can_msg_geo_sync_ptr, &controller_sync_msg);
        if(status) {
            geo_sync = true;
            master_ack_msg.ack_geo = ACK;
        }
    }

    synced = (motorio_sync && bluetooth_sync && geo_sync && sensor_sync);

    if( synced )
    {
        can_msg_t can_ack_msg;
        can_ack_msg.msg_id = MASTER_SYNC_ACK_ID;
        can_ack_msg.frame_fields.is_29bit = false;
        can_ack_msg.frame_fields.data_len = sizeof(master_ack_msg);       // Send 8 bytes
        memcpy( (void *)&(can_ack_msg.data.qword), (void *)&master_ack_msg, sizeof(master_ack_msg) ); // Write all 8 bytes of data at once


        status = CAN_tx(MASTER_CNTL_CANBUS, &can_ack_msg, portMAX_DELAY);
        if( !status ){
            LOG_ERROR("ERROR!!! Master controller: Unable to send Sync Ack CAN message\n");
            LE.on(1);
        }

        else
            LE.off(1);
    }

    return synced;
}

static void bus_off_cb( uint32_t ICR_data )
{
    CAN_reset_bus(MASTER_CNTL_CANBUS);
}

static void data_ovr_cb( uint32_t ICR_data )
{
    CAN_reset_bus(MASTER_CNTL_CANBUS);
}

void check_heartbeat( void ) {

    static uint32_t bluetooth_hb_miss;
    static uint32_t geo_hb_miss;
    static uint32_t sensor_hb_miss;
    static uint32_t motor_hb_miss;

    can_fullcan_msg_t *can_msg_bluetooth_hb_ptr = NULL;
    can_fullcan_msg_t *can_msg_motorio_hb_ptr = NULL;
    can_fullcan_msg_t *can_msg_sensor_hb_ptr = NULL;
    can_fullcan_msg_t *can_msg_geo_hb_ptr = NULL;
    can_fullcan_msg_t hb_msg;

    rst_msg master_rst_msg;

    master_rst_msg.reset_bluetooth = NORESET;
    master_rst_msg.reset_geo = NORESET;
    master_rst_msg.reset_motorio = NORESET;
    master_rst_msg.reset_sensor = NORESET;

    bool status = false;

    can_msg_bluetooth_hb_ptr = CAN_fullcan_get_entry_ptr(can_id_bluetooth_hb);
    can_msg_motorio_hb_ptr = CAN_fullcan_get_entry_ptr(can_id_motor_hb);
    can_msg_geo_hb_ptr = CAN_fullcan_get_entry_ptr(can_id_geo_hb);
    can_msg_sensor_hb_ptr = CAN_fullcan_get_entry_ptr(can_id_sensor_hb);

    if( bluetooth_sync )
    {
        status = CAN_fullcan_read_msg_copy(can_msg_bluetooth_hb_ptr, &hb_msg);
        if( !status ) {
            LOG_ERROR("Missed a Heartbeat from Bluetooth\n");

            bluetooth_hb_miss++;
            if( bluetooth_hb_miss >= MASTER_BT_HB_THRESH ){
                LOG_ERROR("Missed bluetooth heart-beats too many times\n");
                bluetooth_sync = false;
                master_rst_msg.reset_bluetooth = RESET;

                // We have reset the controller, so reset the counter too
                bluetooth_hb_miss = 0;
            }
        }

        else{
            bluetooth_hb_miss = 0;
        }
    }



    if( geo_sync )
    {
        status = CAN_fullcan_read_msg_copy(can_msg_geo_hb_ptr, &hb_msg);
        if( !status ) {
            LOG_ERROR("Missed a Heartbeat from Geo Controller\n");

            geo_hb_miss++;
            if( geo_hb_miss >= MASTER_GEO_HB_THRESH ){
                LOG_ERROR("Missed Geo heart-beats too many times\n");
                geo_sync = false;
                master_rst_msg.reset_geo = RESET;
                geo_hb_miss = 0;
            }
        }

        else{
            geo_hb_miss = 0;
        }
    }

    if( motorio_sync )
    {
        status = CAN_fullcan_read_msg_copy(can_msg_motorio_hb_ptr, &hb_msg);
        if( !status ) {
            LOG_ERROR("Missed a Heartbeat from MotorIO\n");

            motor_hb_miss++;
            if( motor_hb_miss >= MASTER_MOTORIO_HB_THRESH ){
                LOG_ERROR("Missed Motorio heart-beats too many times\n");
                motorio_sync = false;
                master_rst_msg.reset_motorio = RESET;
                motor_hb_miss = 0;
            }
        }
    }

    if( sensor_sync )
    {
        status = CAN_fullcan_read_msg_copy(can_msg_sensor_hb_ptr, &hb_msg);
        if( !status ) {
            LOG_ERROR("Missed a Heartbeat from Sensor controller\n");

            sensor_hb_miss++;
            if( sensor_hb_miss >= MASTER_SENSOR_HB_THRESH ){
                LOG_ERROR("Missed Sensor heart-beats too many times\n");
                sensor_sync = false;
                master_rst_msg.reset_sensor = RESET;
                sensor_hb_miss = 0;
            }
        }
    }

    // Check if anybody needs a reset
    if(
        master_rst_msg.reset_bluetooth == RESET ||
        master_rst_msg.reset_geo == RESET ||
        master_rst_msg.reset_motorio == RESET ||
        master_rst_msg.reset_sensor == RESET )
    {
        // Send CAN message to restart
        can_msg_t can_reset_msg;
        can_reset_msg.msg_id = RESET_ID;
        can_reset_msg.frame_fields.is_29bit = false;
        can_reset_msg.frame_fields.data_len = sizeof(rst_msg);
        memcpy(&can_reset_msg.data.qword, &master_rst_msg, sizeof(rst_msg));

        CAN_tx(MASTER_CNTL_CANBUS, &can_reset_msg, MASTER_CNTL_CAN_DELAY);
    }

    if( !motorio_sync || !sensor_sync || !bluetooth_sync || !geo_sync )
    {
        power_up_sync_and_ack();
        LE.on(MASTER_CNTL_NOHB_LED);
    }

    else
        LE.off(MASTER_CNTL_NOHB_LED);

    LE.toggle(MASTER_CNTL_HB_LED);
}
