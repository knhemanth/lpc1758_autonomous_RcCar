/*
 * bluetooth_controller.hpp
 *
 *  Created on: Oct 30, 2015
 *      Author: Anush Shankar
 */

#ifndef L5_APPLICATION_BLUETOOTH_CONTROLLER_HPP_
#define L5_APPLICATION_BLUETOOTH_CONTROLLER_HPP_

#define heart_beat_enable 0
#define synch_disable 1
#define bt_data_len 7000
#define baud_rate 9600
#define bt_task_mem 1000
#define bt_rx_size 700
#define bt_tx_size 64
#define can_mssg_len 8

#if 1
#define PRINT(...) printf(__VA_ARGS__)
#else
#define PRINT(...)
#endif

bool bluetooth_controller_sync(void);
bool bluetooth_controller_heartbeat(void);

#endif /* L5_APPLICATION_BLUETOOTH_CONTROLLER_HPP_ */
