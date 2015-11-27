/*
 * GPS2.hpp
 *
 *  Created on: Nov 26, 2015
 *      Author: Chitrang
 */

#ifndef GPS2_HPP_
#define GPS2_HPP_

#include "scheduler_task.hpp"
#include "uart2.hpp"
#include "can_msg_id.h"
#include "file_logger.h"

#define PMTK_SET_NMEA_UPDATE_10HZ  "$PMTK220,100*2F" //for changing the update rate to 1HZ
#define PMTK_SET_NMEA_UPDATE_200_MILLIHERTZ  "$PMTK220,5000*1B" //for changing to 200KHZ
#define PMTK_API_SET_FIX_CTL_5HZ  "$PMTK300,200,0,0,0,0*2F"
#define PMTK_SET_BAUD_9600 "$PMTK251,9600*17"
#define PMTK_SET_BAUD_57600 "$PMTK251,57600*2C"
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"

#define GPSInterface gps::getInstance() // Singleton

extern geo_location gps_data_dec;
extern uint8_t speed_gps;

class gps : public SingletonTemplate<gps>
{
    public:

        gps();
        int get_long_degree(void);
        float get_long_minute(void);
        int get_lat_degree(void);
        float get_lat_minute(void);
        float get_decimal(int deg, float minute);
        float get_speed_GPS(void);

        friend class SingletonTemplate<gps>;
        friend class GPSTask;

    private:

        Uart2& gpsUart; // Reference to the Uart Module used to communicate to IMU
};

class GPSTask : public scheduler_task
{
    public:
        GPSTask(unsigned int priority):
            scheduler_task("IMUTask", 3000, priority)
    {
            LOG_DEBUG("IMUTask : Initializing Task");
            // Power on the IMU Interface
            GPSInterface;

            LOG_DEBUG("IMUTask : Initiializing Task Done");
    }

    bool run(void *p);
};




bool gps_init(void);
bool gps_data(void);
int get_long_degree(void);
double get_long_minute(void);
int get_lat_degree(void);
double get_lat_minute(void);
float get_decimal(int deg, float minute);
float get_speed_GPS(void);





#endif /* GPS2_HPP_ */
