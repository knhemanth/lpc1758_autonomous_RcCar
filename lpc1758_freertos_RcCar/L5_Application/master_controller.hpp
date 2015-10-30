/*
 * master_controller.hpp
 *
 *  Created on: Oct 29, 2015
 *      Author: Hemanth K N
 */

#ifndef L5_APPLICATION_MASTER_CONTROLLER_HPP_
#define L5_APPLICATION_MASTER_CONTROLLER_HPP_

#define MASTER_LED_INIT_TIME    (500)
#define MASTER_CNTL_CANBUS      (can2)
#define MASTER_CNTL_BAUD        (100)
#define MASTER_CNTL_RXQ         ( 8 )       // We will be sending maximum of 4 messages at a time, doubling q depth
#define MASTER_CNTL_TXQ         ( 8 )       // Not sure if this will be useful in fullCAN

#define DUMMY_ID                ( 0xFFFF )

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

void bus_off_cb( uint32_t ICR_data );
void data_ovr_cb( uint32_t ICR_data );

#endif /* L5_APPLICATION_MASTER_CONTROLLER_HPP_ */
