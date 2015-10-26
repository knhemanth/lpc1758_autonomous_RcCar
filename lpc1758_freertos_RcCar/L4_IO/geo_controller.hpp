/*
 * geo_controller.hpp
 *
 *  Created on: Oct 25, 2015
 *      Author: Hemanth K N
 */

#ifndef L5_APPLICATION_GEO_CONTROLLER_HPP_
#define L5_APPLICATION_GEO_CONTROLLER_HPP_

#include "can.h"
#include "file_logger.h"

#define ENABLE_DBG  0

#define GEO_CNTL_CANBUS     (can2)      // Use can2
#define GEO_CNTL_BAUD       (100)       // 100kbps baud rate
#define GEO_CNTL_CANTXQ     ( 8 )       // Transmit q size 8
#define GEO_CNTL_CANRXQ     ( 8 )       // Receive q size 8

#define LOG(...)    LOG_ERROR(__VA_ARGS__)

#if ENABLE_DBG
#define CUSTOM_DEBUG(...) LOG_DEBUG(__VA_ARGS__)
#else
#define CUSTOM_DEBUG(...)
#endif

void bus_off_cb( uint32_t icr_data );
void data_ovr_cb( uint32_t icr_data );

#endif /* L5_APPLICATION_GEO_CONTROLLER_HPP_ */
