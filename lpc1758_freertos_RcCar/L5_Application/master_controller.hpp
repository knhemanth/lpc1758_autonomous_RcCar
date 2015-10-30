/*
 * master_controller.hpp
 *
 *  Created on: Oct 29, 2015
 *      Author: Hemanth K N
 */

#ifndef L5_APPLICATION_MASTER_CONTROLLER_HPP_
#define L5_APPLICATION_MASTER_CONTROLLER_HPP_

#include "can.h"

#define MASTER_LED_INIT_TIME    (500)
#define MASTER_CNTL_CANBUS      (can2)
#define MASTER_CNTL_BAUD        (100)
#define MASTER_CNTL_RXQ         ( 8 )       // We will be sending maximum of 4 messages at a time, doubling q depth
#define MASTER_CNTL_TXQ         ( 8 )       // Not sure if this will be useful in fullCAN

#define DUMMY_ID                ( 0xFFFF )

#define MASTER_BT_HB_THRESH         ( 128 ) // If bluetooth heartbeat misses too many times, reset
#define MASTER_GEO_HB_THRESH        ( 32 )
#define MASTER_SENSOR_HB_THRESH     ( 1 )  // Heart-beats are checked only once a second. If we don't know obstacles for a second its a problem
#define MASTER_MOTORIO_HB_THRESH    ( 1 )  // Fingers crossed
#define MASTER_TASK_OVERRUN_DELAY   ( 2000 )
#define MASTER_CNTL_CAN_DELAY       ( 0 )
#define MASTER_CNTL_HB_LED          ( 2 )

enum MASTER_ACK {
    NACK = 0,
    ACK
};

/* Global IDs */
extern can_std_id_t can_id_kill;
extern can_std_id_t can_id_motorio;
extern can_std_id_t can_id_sensor;
extern can_std_id_t can_id_bluetooth;
extern can_std_id_t can_id_geo;
extern can_std_id_t can_id_motor_hb;
extern can_std_id_t can_id_sensor_hb;
extern can_std_id_t can_id_bluetooth_hb;
extern can_std_id_t can_id_geo_hb;
extern can_std_id_t can_id_runmode;
extern can_std_id_t can_id_distance;
extern can_std_id_t can_id_chkpt_snd;
extern can_std_id_t can_id_chkpt_data;
extern can_std_id_t can_id_spd_angle;
extern can_std_id_t can_id_loc_data;

/* Function Prototypes */
bool master_controller_init( void );
void check_heartbeat( void );


#endif /* L5_APPLICATION_MASTER_CONTROLLER_HPP_ */
