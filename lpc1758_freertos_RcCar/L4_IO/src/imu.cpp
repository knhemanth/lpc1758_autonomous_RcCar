/*
 * imu.cpp
 *
 *  Created on: Oct 23, 2015
 *      Author: Akshay Vijaykumar
 */

#include "imu.hpp"

/* TO DO:
 * 1. Current Queue size is 1. The producer queues 1 value every 50 ms
 * while the consumer de-queues every 100ms. Modify the size so that every read
 * by the consumer returns the most recent data.
 * */

/* Custom Module Logging for testing IMU Interfacing */
#if 0 // 1 - Enable, 0 - Disable
#define CUSTOM_DEBUG(...) LOG_DEBUG(__VA_ARGS__)
#else
#define CUSTOM_DEBUG(...)
#endif

/* Constructor
 * Initializes UART used to interface with the IMU
 */
imu::imu():
    imuUart(Uart3::getInstance()),
    imuResetPin(P2_5),
    readYawCommand{'#', 'f'},
    queueHandle(xQueueCreate(IMU_INTERNAL_QUEUE_LENGTH, IMU_BUFFER_SIZE))
{
    // Load Command String to Array

    CUSTOM_DEBUG("IMU : readYawCommand = %c%c", readYawCommand[0], readYawCommand[1]);

    // Initialize Uart3
    if( !(imuUart.init(IMU_UART_BAUDRATE, IMU_UART_RX_QUEUE_SIZE, IMU_UART_TX_QUEUE_SIZE)) )
    {
        LOG_ERROR("IMU Uart Init Failed");
    }
    else
    {
        imuUart.setReady(true);
    }

    CUSTOM_DEBUG("IMU : Uart Init Done");

    // Initialize GPIO as output pin for Power
    imuResetPin.setAsOutput();
    imuResetPin.set(PIN_ON);

    CUSTOM_DEBUG("IMU : Reseting IMU Done");

    // Clearning Internal Buffer;
    clearBuffer();

    CUSTOM_DEBUG("IMU : Internal Buffer Cleared");

}

void imu::resetIMU()
{
    imuResetPin.set(PIN_OFF);

    vTaskDelay(IMU_RESET_TIMEOUT);

    imuResetPin.set(PIN_ON);

    return;
}

/* sendYawReadCommand
 * Issues a Read Command to the IMU
 * */
bool imu::sendYawReadCommand(const char * commandcode)
{
    bool retval = true;

    // Sends a 'Command' over Uart3 to IMU.
    if(imuUart.put(commandcode, IMU_WRITE_TIMEOUT))
    {
        CUSTOM_DEBUG("Sending Command Success");
    }
    else
    {
        CUSTOM_DEBUG("Sending Command Failed");
    }

    return retval;
}

/* getYaeReadingString
 * This function reads the Uart Rx Queue for messages received from IMU.
 * The message is posted to a local queue.
 * */
bool imu::getYawReadingString(void)
{
    bool retval = true;

    // Flush Uart
    imuUart.flush();

    // Read contents of UART Rx Queue
    if(imuUart.gets(buffer, IMU_BUFFER_SIZE, IMU_READ_TIMEOUT))
    {
        CUSTOM_DEBUG("IMU : %s", buffer);
        // Write to Local Queue for Producer Consumer Mechanism
        if(xQueueSend(queueHandle, buffer, IMU_INTERNAL_QUEUE_TIMEOUT) == pdTRUE)
        {
            CUSTOM_DEBUG("IMU : Written Data to Internal Queue");

        }
        else
        {
            CUSTOM_DEBUG("IMU : Failed to Write Data to Internal Queue");
        }
    }
    else
    {
        CUSTOM_DEBUG("IMU Read Failed\n");
        retval = false;
    }

    return retval;
}

/* getValue
 * This function would be called by a consumer
 * */
bool imu::getValue(char * external_buffer, unsigned int external_buffer_size)
{
    bool retval = false;

    /* TO DO : Change Queue Size
     * */

    if((xQueueReceive(queueHandle, external_buffer, IMU_INTERNAL_QUEUE_TIMEOUT)) == pdTRUE)
    {
        CUSTOM_DEBUG("getValue : Read from Queue was Successful");
        retval = true;
    }
    else
    {
        CUSTOM_DEBUG("getValue : Could not read from Queue");
    }

    return retval;
}

/* clearBuffer
 * Clears the contents of the internal Buffer
 */
void imu::clearBuffer(void)
{
    memset(buffer, 0, sizeof(buffer));
}

/* IMUTask
 * This task sends a read command and writes the received message on a local queue.
 * This runs every 50ms, updating the value in the local queue.
 * */
bool IMUTask::run(void *p)
{
    // Send Read Command to IMU
    if(IMUInterface.sendYawReadCommand(IMUInterface.readYawCommand))
    {
        // Waits for a response from IMU
        if(IMUInterface.getYawReadingString())
        {
            CUSTOM_DEBUG("IMUTask : Read success");
        }
        else
        {
            CUSTOM_DEBUG("IMUTask : Read fail");
        }
    }

    // This task should now sleep for 50ms.
    vTaskDelayMs(IMUTASK_DELAY);

    // Always returning true
    return true;
}
