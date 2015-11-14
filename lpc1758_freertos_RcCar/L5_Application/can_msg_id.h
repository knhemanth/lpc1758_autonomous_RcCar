#ifndef __CAN_MSG_ID_H__
#define __CAN_MSG_ID_H__

#ifdef __cplusplus
extern "C"{
#endif
#include <stdint.h>
#include <stdbool.h>

// CAN message IDS

#define KILL_SWITCH_ID				(0x00)
#define RESET_ID					(0x01)
#define MASTER_SYNC_ACK_ID			(0x02)
#define MOTORIO_SYNC_ID				(0x03)
#define SENSOR_SYNC_ID				(0x04)
#define BLUETOOTH_SYNC_ID			(0x05)
#define GEO_SYNC_ID					(0x06)
#define MOTORIO_HEARTBEAT_ID		(0x07)
#define SENSOR_HEARTBEAT_ID			(0x08)
#define BLUETOOTH_HEARTBEAT_ID		(0x09)
#define GEO_HEARTBEAT_ID			(0x0A)
#define RUN_MODE_ID					(0x0B)
#define DISTANCE_SENSOR_ID			(0x0C)
#define MOTOR_DIRECTIONS_ID			(0x0D)
#define CHECKPOINT_REQ_ID			(0x0E)
#define CHECKPOINT_SEND_ID			(0x0F)
#define CHECKPOINT_DATA_ID			(0x10)
#define GEO_LOC_UPDATE_ID			(0x11)
#define GEO_SPEED_ANGLE_ID			(0x12)
#define GEO_LOC_DATA_ID				(0x13)
#define LIGHT_BATTERY_SENSOR_ID		(0x14)

/*
 *	Data structures for CAN message IDs
 *
 *	The following messages have no data fields
 *
 *	1. Kill switch
 *	2. Reset
 *	3. MOTORIO_SYNC
 *	4. SENSOR_SYNC
 *	5. BLUETOOTH_SYNC
 *	6. GEO_SYNC
 *	7. CHECKPOINT_REQ_ID
 */


/* 
 * Master acknowledges each controller
 * Sent by: Master controller
 * Received by: All controllers
 */
typedef struct master_sync_ack{

	uint8_t ack_motorio;		// Acknowledge motorio controller
	uint8_t ack_sensor;			// Acknowledge sensor controller
	uint8_t ack_geo;			// Acknowledge geo controller
	uint8_t ack_bluetooth;		// Acknowledge bluetooth module

}__attribute__((__packed__)) master_sync;


/* 
 * Heart-beat message for each controller 
 * Sent by: All controllers
 * Received by: Master controller
 */
typedef struct heart_beat{

	uint8_t counter;			// Continuity counter that increments with every beat

}__attribute__((__packed__)) heart_beat;

/* 
 * Run mode for the vehicle. 
 * TODO: Define enum for each mode
 *
 * Sent by: Bluetooth module [ original data from the Android app ]
 * Received by: Master controller
 */

typedef struct run_mode{

	uint8_t mode;				// Run modes - 1. Navigation mode, 2. Free run mode and 3. Manual mode

}__attribute__((__packed__)) run_mode;

/*
 *	Ultrasonic sensor readings for obstacle avoidance
 *	Sent by: Sensor controller
 *	Received by: Master controller
 */
typedef struct distance_sensor{

	uint8_t front_left;			// Front left sensor reading
	uint8_t front_right;		// Front right sensor reading
	uint8_t front_center;		// Front centre sensor reading
	uint8_t left;				// Left sensor reading
	uint8_t right;				// Right sensor reading
	uint8_t back;				// Back sensor reading

}__attribute__((__packed__)) dist_sensor;

/* 
 * Indication to motor IO for driving.
 *
 * Sent by: Master controller 
 * Received by: MotorIO controller
 */
typedef struct motor_direction{

	uint8_t speed;				// Indicate speed for DC motor
	uint8_t turn;				// Indicate turn angle for servo motor

}__attribute__((__packed__)) motor_direction;

/* 
 * Indicate number of check-points in the route. Then send each check point using check-point data 
 *
 * Sent by: Bluetooth module [ After receiving data from Android app ]
 * Received by: Master controller
 */
typedef struct checkpoint_send{

	uint32_t num_of_points;		// Number of check-points to be loaded

}__attribute__((__packed__)) chk_point_snd;

/* 
 * Check-point data sent for each check-point 
 *
 * Sent by: Bluetooth module
 * Received by: Master controller
 */
typedef struct checkpoint_data{

	float latitude;
	float longitude;

}__attribute__((__packed__)) chk_point_data;

/* 
 * Send destination lattitude and longitude to the geo controller. Here destination will be the next check-point 
 *
 * Sent by: Master controller
 * Received by: Geo controller
 */
typedef chk_point_data geo_loc;	// use geo_loc instead of chk_point_data

/* 
 * Indicate speed measured by GPS, the current heading and bearing from IMU 
 *
 * Sent by: Geo controller
 * Received by: Master controller and IO controller
 */
typedef struct geo_speed_angle{

	uint8_t speed;				// Speed as measured by the GPS sensor
	uint16_t heading;			// Heading from the Geo controller
	uint16_t bearing;			// Bearing calculated by the Geo controller

}__attribute__((__packed__)) geo_spd_angle;

/* 
 * Light and battery sensor readings 
 *
 * Sent by: Sensor controller
 * Received by: IO controller
 */
typedef struct light_battery_sensor{

	uint8_t light_sensor;		// Light sensor reading
	uint8_t batt_sensor;		// Battery level sensor reading

}__attribute__((__packed__)) lght_batt_reading;

#ifdef __cplusplus
}
#endif
#endif

