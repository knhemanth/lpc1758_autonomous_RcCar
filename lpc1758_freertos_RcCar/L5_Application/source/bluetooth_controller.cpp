/*
 * bluetooth_controller.cpp
 *
 *  Created on: Oct 30, 2015
 *      Author: Anush Shankar
 */

#include "stdio.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "can.h"
#include "bluetooth_controller.hpp"
#include "can_msg_id.h"
#include "file_logger.h"
#include "soft_timer.hpp"

#define can_baud_kbps 100
#define can_rx_queue_size 512
#define can_tx_queue_size 512
#define BT_SYNC_TIME 500
#define BT_CNTRL_TIMEOUT_CAN 10

static can_msg_t tx_mssg;
static can_msg_t master_rx_mssg;
static bool can_bus_off_flag = false;

void bus_off_cb( uint32_t icr_data )
{
    can_bus_off_flag = true;
}

void data_ovr_cb( uint32_t icr_data )
{

}

bool bluetooth_controller_sync()
{
    bool can_init_stat = false;

    bool sync_tx_status = false;

    bool sync_ack_status = false;

    SoftTimer can_rx_timer(BT_SYNC_TIME);

    //Initialize can bus for sync frame transmission
    can_init_stat = CAN_init(can2, can_baud_kbps, can_rx_queue_size, can_tx_queue_size, bus_off_cb, data_ovr_cb);
    puts("can init started\n");

    if(!can_init_stat)
    {
        LOG_ERROR("Bluetooth controller CAN bus could not be initialiazed");
        return can_init_stat;
    }

    puts("\ncan init successful");
    CAN_bypass_filter_accept_all_msgs();
    CAN_reset_bus(can2);

    tx_mssg.msg_id = BLUETOOTH_SYNC_ID;
    tx_mssg.frame_fields.data_len = 0;
    tx_mssg.frame_fields.is_29bit = 0;
    tx_mssg.data.qword = 0;

    do
    {
        sync_tx_status = CAN_tx(can2, &tx_mssg,BT_CNTRL_TIMEOUT_CAN);
        puts("\ncan tx started");

        if(!sync_tx_status)
        {
            LOG_ERROR("Bluetooth can message cannot be sent");
        }
        if(sync_tx_status)
            puts("\ncan tx successful no errors");

        can_rx_timer.restart();
        while(!can_rx_timer.expired());

        sync_ack_status = CAN_rx(can2,&master_rx_mssg, BT_CNTRL_TIMEOUT_CAN);

        puts("\ncan rx started");
        if(sync_ack_status)
        {
            puts("\ncan rx successful");
            if(master_rx_mssg.msg_id == MASTER_SYNC_ACK_ID)
                sync_ack_status = true;
        }

    }while(sync_ack_status == false);

    puts("\nack received  successful");
    return sync_ack_status;
}






