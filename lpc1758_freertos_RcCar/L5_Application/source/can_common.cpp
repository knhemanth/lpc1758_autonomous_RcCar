/*
 * File         : can_common.cpp
 * Author       : Dhruv Kakadiya
 * Date         : September 13, 2015
 */

#include "can_common.hpp"

can_msg_t no_motor_msg,tmp_can_msg,received_msg;
extern can_msg_t motor_msg;
int no_motor_msg_count=0;

static bool bus_off_state = false;
static bool powerup_sync_motor_io_controller( void );

static void can_bus_off_callback(uint32_t dummy)
{
    bus_off_state = true;
    LE.set(BUS_OFF_LED, true);
}

void data_over_callback( uint32_t icr_data )
{
    //LOG_WARN("WARNING!!! GEO Controller CAN bus overflow!!!");
    CAN_reset_bus(MOTORIO_CNTL_CANBUS);
}

void check_bus_off(void)
{
    if(bus_off_state)
    {
        CAN_reset_bus(MOTORIO_CNTL_CANBUS);
        bus_off_state = false;
        LE.set(BUS_OFF_LED, false);
    }
}

bool can_init(void) {
    bool status = false;
    no_motor_msg.data.bytes[0] = (uint8_t) 0 ;      // Stop speed
    no_motor_msg.data.bytes[1] = (uint8_t) 2 ;      // Straight direction

    LE.init();
    status = CAN_init(MOTORIO_CNTL_CANBUS,MOTORIO_CNTL_CANBAUD,MOTORIO_CNTL_CANRXQ,MOTORIO_CNTL_CANTXQ,can_bus_off_callback,data_over_callback);
    if( !status )
    {
        LOG_ERROR("ERROR!!! Cannot initialize MOTORIO controller CAN Bus!!");
        return status;
    }
    LOG_INFO("MOTORIO CAN bus is initialized\n");

    // Setup necessary message filter
    const can_std_id_t slist[] = { CAN_gen_sid(MOTORIO_CNTL_CANBUS, RESET_ID), CAN_gen_sid(MOTORIO_CNTL_CANBUS, MASTER_SYNC_ACK_ID),
                                   CAN_gen_sid(MOTORIO_CNTL_CANBUS, RUN_MODE_ID), CAN_gen_sid(MOTORIO_CNTL_CANBUS, MOTOR_DIRECTIONS_ID)
                                 };
    CAN_setup_filter(slist, 4, NULL, 0, NULL, 0, NULL, 0);
    CAN_reset_bus(MOTORIO_CNTL_CANBUS);

    // Sync with the master controller by sending power_up_sync
    status = powerup_sync_motor_io_controller();
    return status;
}

// Function to check if CAN message data value is the same as previous message then return false otherwise return true
bool test_data(can_msg_t can_message){
    if(can_message.data.qword == tmp_can_msg.data.qword){
        return false;
    }
    else {
        tmp_can_msg = can_message;
        return true;
    }
}

// Transmit data over CAN
bool transmit_data(can_msg_t transmit_msg){
    transmit_msg.frame_fields.is_29bit = 0;
    transmit_msg.frame_fields.data_len = 8;
    if(test_data(transmit_msg)) {
        if(false == CAN_tx(MOTORIO_CNTL_CANBUS, &transmit_msg, 0)) {
            //SET_ERROR(ERROR_TX_FAILED);
            LOG_ERROR("CAN_tx failed\n");
            return false;
        }
        else
            return true;
    }
    else
        return false;
}

// Receive data over CAN
bool receive_data(){

    if(false == CAN_rx(MOTORIO_CNTL_CANBUS, &received_msg, 0)) {
        //SET_ERROR(ERROR_TX_FAILED);
        //LOG_ERROR("CAN_rx failed\n");
        motor_msg = no_motor_msg;
        no_motor_msg_count++;
        return false;
    }
    else {
        no_motor_msg_count = 0;
        if (received_msg.msg_id == MOTOR_DIRECTIONS_ID) {
            motor_msg = received_msg;
        }
        else if (received_msg.msg_id == RESET_ID) {
            motor_io_get_master_reset();
        }
        else if (received_msg.msg_id == MASTER_SYNC_ACK_ID) {
            // MotorIO controller get sync with Master controller
        }
        else if (received_msg.msg_id == RUN_MODE_ID) {
            // DC motor's base speed is get changed
        }
        return true;
    }
}

// Send Motor_IO heartbeat to master
void motor_io_send_heartbeat( void )
{
    // Heart Beat to Master
    can_msg_t heartbeat_motor_io_msg; // Can Message
    bool can_status = false;

    heartbeat_motor_io_msg.msg_id = MOTORIO_HEARTBEAT_ID; // MOTORIO Heartbeat ID
    heartbeat_motor_io_msg.frame_fields.is_29bit = 0;
    heartbeat_motor_io_msg.frame_fields.data_len = 0;

    can_status = CAN_tx(MOTORIO_CNTL_CANBUS, &heartbeat_motor_io_msg, MOTORIO_CNTL_CAN_TIMEOUT);

    if( !can_status )
    {
        LOG_ERROR("ERROR!!! MotorIO controller Heartbeat message not sent!!");
        LE.off(MOTORIO_HB_LED);
    }
    else
    {
        LE.toggle(MOTORIO_HB_LED);
    }

    if(bus_off_state) {
        CAN_reset_bus(MOTORIO_CNTL_CANBUS);
        bus_off_state = false;
    }
}

// If get reset command from Master then reboot the whole system
void motor_io_get_master_reset( void )
{
    LOG_ERROR("ERROR!!! Received a reset request from master\n");
    sys_reboot();
}

// PowerUp Sync function for Motor_IO Controller
bool powerup_sync_motor_io_controller( void )
{
    can_msg_t motor_io_sync_msg;
    uint8_t sync_miss_count = 0;
    master_sync* sync_msg;
    bool sync_ack = false;
    bool status = false;

    SoftTimer can_rx_timer(MOTORIO_CNTL_SYNC_TIME);

    motor_io_sync_msg.msg_id = MOTORIO_SYNC_ID;
    motor_io_sync_msg.frame_fields.is_29bit = 0;
    motor_io_sync_msg.frame_fields.data_len = 0;     // No data
    motor_io_sync_msg.data.qword = 0;

    do
    {
        // Send sync message
        status = CAN_tx(MOTORIO_CNTL_CANBUS, &motor_io_sync_msg, MOTORIO_CNTL_CAN_TIMEOUT);

        if( !status )
        {
            LOG_ERROR("ERROR!!! Unable to send MotorIO controller sync message\n");
            LE.on(MOTORIO_CAN_ERR_LED);
        }

        else
            LE.off(MOTORIO_CAN_ERR_LED);

        // No need to delay here
        // XXX: Cannot use FreeRTOS functions until the OS runs
        // vTaskDelayMs(MOTORIO_CNTL_SYNC_TIME);


        can_rx_timer.restart();
        while( !can_rx_timer.expired() );


        // Check for ack
        status = CAN_rx(MOTORIO_CNTL_CANBUS, &received_msg, 0);

        // XXX: Since OS is not up, maybe you can use SoftTime (polling timer)
        if( status )
        {
            // We have a new message. Check if MotorIO is acked

            // XXX: Suggest "shared" structures rather than memcpy
            sync_msg = (master_sync *)&received_msg.data.bytes[0];
            if( sync_msg->ack_geo )
                sync_ack = true;

        }

        sync_miss_count++;

    }while( ( sync_ack == false ) );    // try forever

#if 0
    if( !sync_ack )
        LOG_ERROR("ERROR!!! Sync timeout from MotorIO controller. Stalling!!!\n");
#endif

    return sync_ack;
}
