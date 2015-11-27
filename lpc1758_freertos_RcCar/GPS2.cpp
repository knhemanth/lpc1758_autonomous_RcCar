/*
 * GPS2.cpp
 *
 *  Created on: Nov 26, 2015
 *      Author: Chitrang
 */
#include "GPS.hpp"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "utilities.h"
#include "can_msg_id.h"
#include "GPS2.hpp"

geo_location gps_data_dec;
uint8_t speed_gps;


gps::gps(): gpsUart(Uart2::getInstance())
{
    gpsUart.init(9600,100,100);
    gpsUart.putline(PMTK_SET_BAUD_9600,1);
    gpsUart.putline(PMTK_SET_NMEA_UPDATE_10HZ,1);
    //  uart2_ref.putline(PMTK_API_SET_FIX_CTL_5HZ,99999999);
    //   uart2_ref.putline(PMTK_SET_BAUD_57600,99999999);
    gpsUart.putline(PMTK_SET_NMEA_OUTPUT_RMCONLY,1);
    // uart2_ref.putChar(clear_byte1,9999999);
    //uart2_ref.putChar(clear_byte2,9999999);
}

float gps::get_decimal(int deg, float minute){
    return deg + minute/60;

}

int gps::get_lat_degree(void){

    char a;
        char buffer[10]={'\0'};
        while(a!='C'){
            gpsUart.getChar(&a,1);
        }
        while(a!='A'){
            gpsUart.getChar(&a,1);
         //   if(a == 'V')
          //      return 0;
        }

        gpsUart.getChar(&a,1);
        int count=0;
        while(count<2){
            gpsUart.getChar(&a,1);
            buffer[count++]=a;
        }

        return atoi(buffer);
}

float gps::get_lat_minute(void){
    char a;
    char buffer[10]={'\0'};
    int count=0;
    while(count<7){
        gpsUart.getChar(&a,1);
        buffer[count++]=a;
    }
    return atof(buffer);
}

int gps::get_long_degree(void){
    char a;
    char buffer[10]={'\0'};
    while(a!='N'){
        gpsUart.getChar(&a,1);
    }
    gpsUart.getChar(&a,1);
    int count=0;
    while(count<3){
        gpsUart.getChar(&a,1);
        buffer[count++]=a;
    }
    return atoi(buffer);
}

float gps::get_long_minute(void){
    char a;
    char buffer[10]={'\0'};
    int count=0;
    while(count<7){
        gpsUart.getChar(&a,1);
        buffer[count++]=a;
    }
    return atof(buffer);
}

float gps::get_speed_GPS(void){
    char a;
    char buffer[10] = {'\0'};
    while(a!='W'){
            gpsUart.getChar(&a,1);
        }
    gpsUart.getChar(&a,0);
    int count=0;
        while(count<4){
            gpsUart.getChar(&a,1);
            buffer[count++]=a;
        }
        return atof(buffer);
}

bool GPSTask::run(void *p)
{
    // Send Read Command to IMU


    static int lat_degree = 0;
    static float lat_minute = 0;
    static int long_degree = 0;
    static float long_minute = 0;
    static float lat_dec;
    static float long_dec;

    lat_degree = GPSInterface.get_lat_degree();
    lat_minute = GPSInterface.get_lat_minute();
    long_degree =GPSInterface.get_long_degree();
    long_minute =GPSInterface.get_long_minute();
    speed_gps = (uint8_t)(1.150779)*GPSInterface.get_speed_GPS();

    lat_dec = GPSInterface.get_decimal(lat_degree, lat_minute);
    long_dec = GPSInterface.get_decimal(long_degree, long_minute);

    gps_data_dec.latitude = lat_dec;
    gps_data_dec.longitude = (-1)*long_dec;

   // vTaskDelayMs(10);

    // Always returning true
    return true;
}
