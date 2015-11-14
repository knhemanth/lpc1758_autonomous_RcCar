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
#include <stdio.h>

/* Static ID variables - Should be used as read only */
static can_std_id_t can_id_kill;
static can_std_id_t can_id_motorio;
static can_std_id_t can_id_sensor;
static can_std_id_t can_id_bluetooth;
static can_std_id_t can_id_geo;
static can_std_id_t can_id_motor_hb;
static can_std_id_t can_id_sensor_hb;
static can_std_id_t can_id_bluetooth_hb;
static can_std_id_t can_id_geo_hb;
static can_std_id_t can_id_runmode;
static can_std_id_t can_id_distance;
static can_std_id_t can_id_chkpt_snd;
static can_std_id_t can_id_chkpt_data;
static can_std_id_t can_id_spd_angle;
static can_std_id_t can_id_loc_data;

/* Static variables */
static bool motorio_sync = false;
static bool bluetooth_sync = false;
static bool geo_sync = false;
static bool sensor_sync = false;

/* Static function prototypes */
static bool power_up_sync_and_ack( void );
static void bus_off_cb( uint32_t ICR_data );
static void data_ovr_cb( uint32_t ICR_data );

static bool bus_off_status = false;

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

#if HEARTBEAT
    while( !( power_up_sync_and_ack() ) );
#endif

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

void check_bus_off(void)
{
    if(bus_off_status)
    {
        CAN_reset_bus(MASTER_CNTL_CANBUS);
        bus_off_status = false;
    }

    return;
}

static void bus_off_cb( uint32_t ICR_data )
{
    bus_off_status = true;
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


    /*
     * Check the heart-beat message. Two things can happen
     *
     * 1. Master controller has requested one of the controllers to reset.
     *    The controller has missed the reset message due to whatever caused it
     *    to stop its heart-beats. Now it gets back up [without resetting] and
     *    starts sending the heart-beats. So master at this point should say, you are
     *    out of sync and missed your reset message. So reset again.
     *
     *    ???: OR should we just take the heart-beat and say he's synced?
     *
     * 2. Master has not requested a reset of any controller. So he checks
     *    the heart-beat and proceeds further.
     */

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

        if( bluetooth_sync ){
            bluetooth_hb_miss = 0;
        }

        else    // Here we are getting heart-beats in spite of being out of sync
            master_rst_msg.reset_bluetooth = RESET;
    }



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
        if( geo_sync ){
            geo_hb_miss = 0;
        }

        else
            master_rst_msg.reset_geo = RESET;
    }

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

    else{
        if( !motorio_sync ){
            master_rst_msg.reset_motorio = RESET;
        }
    }

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

    else{
        if( !sensor_sync ){
            master_rst_msg.reset_sensor = RESET;
        }
    }

    // Check if anybody needs a reset
    if(
        master_rst_msg.reset_bluetooth == RESET ||
        master_rst_msg.reset_geo == RESET ||
        master_rst_msg.reset_motorio == RESET ||
        master_rst_msg.reset_sensor == RESET )
    {
        // Send CAN message to reset the controllers
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
    {
        LE.off(MASTER_CNTL_NOHB_LED);
        LE.toggle(MASTER_CNTL_HB_LED);
    }
}

bool avoid_obstacle(void)
{
    bool status = false;
    dist_sensor *sensor_msg;
    motor_direction motor_data;
    can_fullcan_msg_t *can_msg_sensor_ptr = NULL;
    can_fullcan_msg_t can_msg_sensor_data;

    uint32_t left_sensor = 0;
    uint32_t right_sensor = 0;
    uint32_t front_sensor = 0;

    // Read Sensor Values for Obstacle Zones - Happens every 100Hz
    can_msg_sensor_ptr = CAN_fullcan_get_entry_ptr(can_id_distance);

    status = CAN_fullcan_read_msg_copy(can_msg_sensor_ptr, &can_msg_sensor_data);
    if(!status)
    {
        //LOG_ERROR("Error!!! : Dids not     Sensor Data\n");
        LE.on(MASTER_CAN_ERR_LED);
        return status;
    }
    else
    {
        LE.off(MASTER_CAN_ERR_LED);
    }

    sensor_msg = (dist_sensor *) &(can_msg_sensor_data.data.bytes[0]);

    // Avoiding only obstacles in the near zones
    // Decide direction and speed for Motor

    left_sensor = sensor_msg->front_left;
    right_sensor = sensor_msg->front_right;
    front_sensor = sensor_msg->front_center;

#if ZONE_CALCULATION

    float left_sensor_value = left_sensor * 1000;
    float right_sensor_value = right_sensor * 1000;
    float middle_sensor_value = front_sensor * 1000;

    // Determine Zone for Each Sensor

    left_sensor = getZoneInformation(left_sensor_value);
    right_sensor = getZoneInformation(right_sensor_value);
    front_sensor = getZoneInformation(middle_sensor_value);

#endif

#if 0 //OBSTACLE_AVOIDANCE ONE

    unsigned int temp_range = MID;

    if( (left_sensor > temp_range) && (right_sensor > temp_range) && (front_sensor > temp_range) )
    {
        // Stop Car
        motor_data.speed = (uint8_t)STOP;
        motor_data.turn = (uint8_t)STRAIGHT;
    }
    else
    {
        // Default
        motor_data.speed = (uint8_t)SLOW;
        motor_data.turn = (uint8_t)STRAIGHT;


        if( left_sensor >= right_sensor )
        {
            if(left_sensor >= temp_range)
            {
                // Send Slight Right to Motor
                motor_data.turn = (uint8_t)RIGHT;
            }
            else
            {
                // Send Full Right to Motor
                motor_data.turn = (uint8_t)S_RIGHT;
            }
        }
        else if( right_sensor >= left_sensor )
        {
            if(right_sensor >= temp_range)
            {
                // Send Slight Left to Motor
                motor_data.turn = (uint8_t)LEFT;
            }
            else
            {
                // Send Full Left to Motor
                motor_data.turn = (uint8_t)S_LEFT;
            }
        }
        else if(( front_sensor >= temp_range))
        {
            if( left_sensor >= right_sensor)
            {
                // Move Slight Right
                if( left_sensor >= temp_range)
                    motor_data.turn = (uint8_t)RIGHT;
                else
                    motor_data.turn = (uint8_t)S_RIGHT;
            }
            else
            {
                // Move Slight Left
                if( right_sensor >= temp_range )
                    motor_data.turn = (uint8_t) LEFT;

                else
                    motor_data.turn = (uint8_t) S_LEFT;
            }
        }
    }

#endif

#if OBSTACLE_AVOIDANCE

    if((left_sensor == NEAR) || (front_sensor == NEAR) || (right_sensor == NEAR))
    {
        motor_data.speed = (uint8_t) SLOW;

        if((left_sensor == NEAR) && (front_sensor == NEAR) && (right_sensor == NEAR))
        {
            motor_data.turn = (uint8_t) STRAIGHT;
            motor_data.speed = (uint8_t) STOP;
        }
        else if((left_sensor != NEAR) && (front_sensor != NEAR) && (right_sensor == NEAR))
        {
            motor_data.turn = (uint8_t) LEFT;
        }
        else if((left_sensor != NEAR) && (front_sensor == NEAR) && (right_sensor != NEAR))
        {
            motor_data.turn = (uint8_t) STRAIGHT;
            motor_data.speed = (uint8_t) STOP;
        }
        else if((left_sensor != NEAR) && (front_sensor == NEAR) && (right_sensor == NEAR))
        {
            motor_data.turn = (uint8_t) LEFT;
        }
        else if((left_sensor == NEAR) && (front_sensor != NEAR) && (right_sensor != NEAR))
        {
            motor_data.turn = (uint8_t) RIGHT;
        }
        else if((left_sensor == NEAR ) && (front_sensor != NEAR) && (right_sensor == NEAR))
        {
            motor_data.turn = (uint8_t) STRAIGHT;
            motor_data.speed = (uint8_t) STOP;
        }
        else if((left_sensor == NEAR) && (front_sensor == NEAR) && (right_sensor != NEAR))
        {
            motor_data.turn = (uint8_t) RIGHT;
        }
    }
    else if((left_sensor == MID) || (front_sensor == MID) || (right_sensor == MID))
    {
        motor_data.speed = (uint8_t) SLOW;

        if((left_sensor != MID) && (front_sensor != MID) &&(right_sensor == MID))
        {
            motor_data.turn = (uint8_t) S_LEFT;
        }
        else if((left_sensor != MID) && (front_sensor == MID) &&(right_sensor != MID))
        {
            if(right_sensor > left_sensor)
            {
                motor_data.turn = (uint8_t) S_LEFT;
            }
            else if(right_sensor < left_sensor)
            {
                motor_data.turn = (uint8_t) S_RIGHT;
            }
            else
            {
                motor_data.turn = (uint8_t) STRAIGHT;
            }
        }
        else if((left_sensor != MID) && (front_sensor == MID) &&(right_sensor == MID))
        {
            motor_data.turn = (uint8_t) S_LEFT;
        }
        else if((left_sensor == MID) && (front_sensor != MID) &&(right_sensor != MID))
        {
            motor_data.turn = (uint8_t) S_RIGHT;
        }
        else if((left_sensor == MID) && (front_sensor != MID) &&(right_sensor == MID))
        {
            motor_data.turn = (uint8_t) STRAIGHT;
        }
        else if((left_sensor == MID) && (front_sensor == MID) &&(right_sensor != MID))
        {
            motor_data.turn = (uint8_t) S_RIGHT;
        }
        else if((left_sensor == MID) && (front_sensor == MID) &&(right_sensor == MID))
        {
            motor_data.turn = (uint8_t) STRAIGHT;
        }
    }
    else
    {
        motor_data.turn = (uint8_t) STRAIGHT;
        motor_data.speed = (uint8_t) NORMAL;
    }

#endif

    // Send Can messages only if Motor and Sensor Controllers are in Sync with the Master
#if HEARTBEAT
    if((motorio_sync) && (sensor_sync))
    {
#endif
        // Send Command over CAN to Motor
        can_msg_t can_motor_msg;
        can_motor_msg.msg_id = MOTOR_DIRECTIONS_ID;
        can_motor_msg.frame_fields.is_29bit = false;
        can_motor_msg.frame_fields.data_len = sizeof(motor_direction);
        memcpy(&can_motor_msg.data.qword, &motor_data, sizeof(motor_direction));

        CAN_tx(MASTER_CNTL_CANBUS, &can_motor_msg, MASTER_CNTL_CAN_DELAY);

#if DEBUG
        printf("%c %c %c | ", printRange(sensor_msg->front_left), printRange(sensor_msg->front_center), printRange(sensor_msg->front_right));
        printf("%c %c\n", printMotorSpeed((uint8_t) motor_data.speed), printMotorDirection((uint8_t) motor_data.turn));
#endif

#if HEARTBEAT
    }
#endif
    return true;
}

#if DEBUG
char printRange(uint8_t  zone)
{
    if(zone == FAR)
    {
        return 'F';
    }
    else if(zone == MID)
    {
        return 'M';
    }
    else if(zone == NEAR)
    {
        return 'N';
    }

    return 'P';
}

char printMotorDirection(uint8_t data)
{
    char ch = '.';

    switch(data)
    {
        case LEFT :
            ch = 'L';
            break;

        case S_LEFT :
            ch = 'l';
            break;

        case STRAIGHT:
            ch = 'S';
            break;

        case RIGHT :
            ch = 'R';
            break;

        case S_RIGHT :
            ch = 'r';
            break;

        case BACK :
            ch = 'B';
            break;
    }

    return ch;
}

char printMotorSpeed(uint8_t speed)
{
    char ch = '.';

    switch(speed)
    {
        case STOP :
            ch = 'S';
            break;

        case SLOW :
            ch = 's';
            break;

        case NORMAL :
            ch = 'N';
            break;

        case FAST :
            ch = 'F';
            break;

        case TURBO :
            ch = 'T';
            break;
    }

    return ch;
}
#endif

#if ZONE_CALCULATION
int getZoneInformation(float value)
{
    if(value < ZONE_NEAR_THRESHOLD)
    {
        return NEAR;
    }
    else if((value >= ZONE_NEAR_THRESHOLD) && (value < ZONE_MID_THRESHOLD))
    {
        return MID;
    }
    else if((value >= ZONE_MID_THRESHOLD) && (value < ZONE_FAR_THRESHOLD))
    {
        return FAR;
    }
    else
    {
        return NO_OBSTACLE;
    }
}
#endif

#if BT_APP
bool update_from_app(void)
{
    bool status = false;
    static bool command_from_app = false;
    can_fullcan_msg_t *can_msg_app_ptr = NULL;
    can_fullcan_msg_t can_msg_app_data;

    // Temporary App is sent via the Kill Switch Message ID
    can_msg_app_ptr = CAN_fullcan_get_entry_ptr(can_id_kill);

    status = CAN_fullcan_read_msg_copy(can_msg_app_ptr, &can_msg_app_data);

    // If an updated message from Bluetooth Controller is received, change state
    // Else, return previous state
    if(status)
    {
        command_from_app = (bool) (can_msg_app_data.data.bytes[0]);

        if(!command_from_app)
        {
            // Send Stop Command over CAN to Motor in case command_from_app is false
            motor_direction motor_data;
            can_msg_t can_motor_msg;

            motor_data.speed = STOP;
            motor_data.turn = STRAIGHT;

            can_motor_msg.msg_id = MOTOR_DIRECTIONS_ID;
            can_motor_msg.frame_fields.is_29bit = false;
            can_motor_msg.frame_fields.data_len = sizeof(motor_direction);
            memcpy(&can_motor_msg.data.qword, &motor_data, sizeof(can_motor_msg));

            CAN_tx(MASTER_CNTL_CANBUS, &can_motor_msg, MASTER_CNTL_CAN_DELAY);
        }

        LE.set(1, command_from_app);
    }

    return command_from_app;
}
#endif
