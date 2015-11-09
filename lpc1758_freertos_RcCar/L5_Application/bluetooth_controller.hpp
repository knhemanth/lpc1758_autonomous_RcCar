/*
 * bluetooth_controller.hpp
 *
 *  Created on: Oct 30, 2015
 *      Author: Anush Shankar
 */

#ifndef L5_APPLICATION_BLUETOOTH_CONTROLLER_HPP_
#define L5_APPLICATION_BLUETOOTH_CONTROLLER_HPP_

#define heart_beat_enable 0
#define test_can_bt 0
#define synch_disable 1

bool bluetooth_controller_sync(void);
bool bluetooth_controller_heartbeat(void);

#endif /* L5_APPLICATION_BLUETOOTH_CONTROLLER_HPP_ */
