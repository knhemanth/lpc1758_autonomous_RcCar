/*
 * motor_controller.cpp
 *
 *  Created on: Nov 1, 2015
 *      Author: Dhruv
 */

#include "motor_controller.hpp"
#include "_can_dbc/generated_motorio_code.h"
#include "adc0.h"

#define DC_ON                               1
#define SERVO_ON                            1
#define SPEED_FEEDBACK                      1
#define LIGHT_SENSOR_CHANNEL                4
#define RPM_DEAD_SECOND_COUNT               4

#define CAN_ERROR_LED                       4
#define SPEED_FACTOR_ERROR                  4

#define SPEED_VAR_FACTOR                    0.05
#define HARD_LEFT                           9.0
#define S_LEFT                              8.0
#define HARD_RIGHT                          5.7
#define S_RIGHT                             6.5
#define STRAIGHT                            7.5

#define DC_STOP                             7.0

#define DC_THRESH_SLOW                      6.1
#define DC_THRESH_NORMAL                    6.0
#define DC_THRESH_TURBO                     5.9

// For RPM sensor configuration
#define ADC04_PINSELECT_VALUE               (  0x00000003 )
#define ADC04_PINSELECT_SHIFT               ( 28 )

extern DRIVER_TX_MOTORIO_DIRECTION_t motor_msg;
extern QueueHandle_t g_adc_result_queue;

float dc_stop = 7.0;
extern float dc_slow;
extern float dc_normal;
extern float dc_turbo;

bool white_mark = false;
int white_mark_count = 0;
int check_time = 0;
float speed_factor = DC_STOP;
int desired_count = 0;
int dc_slow_count = 2;
int dc_normal_count = 3;
int dc_turbo_count = 4;
int dead_count = 0;
int light_threshold = 1900;
//As per waveform and real testing(percent range - 6.0(right) - 7.5(center) - 9.3(left))
//As per waveform only(percent range - 5.5(forward) - 8.5(stop) - 10.5(backward))

void rpm_init(void)
{
    LPC_PINCON->PINSEL3 |= (ADC04_PINSELECT_VALUE << ADC04_PINSELECT_SHIFT);
    adc0_init();
}

void motor_init(void)
{

    LD.init();
    int c=0;
    float factor = 5.5;
    MotorControl.setServo(STRAIGHT);
    delay_ms(100);

    while(factor<9.3)
    {
        MotorControl.setServo(factor);
        factor+=0.1;
        delay_ms(50);
    }

    while(factor>5.5)
    {
        MotorControl.setServo(factor);
        factor-=0.1;
        delay_ms(50);
    }
    MotorControl.setServo(STRAIGHT); // Set servo straight again

    while(c < 20)
    {
        MotorControl.setDC(DC_STOP);
        delay_ms(50);
        c++;
    }
    rpm_init();
    //MotorControl.setDC(6.10);
    //delay_ms(5000);
    //MotorControl.setDC(DC_STOP);
    //delay_ms(50);
    printf("motor_init is done\n");
}

void set_motors_pwm(void)
{
    motor_direction md; // Copy received CAN msg into proper data structure
    md.turn = motor_msg.MOTORIO_DIRECTION_turn_cmd;
    md.speed = motor_msg.MOTORIO_DIRECTION_speed_cmd;

#if SERVO_ON
    if (md.turn == left)
    { // turn hard left
        MotorControl.setServo(HARD_LEFT);
    }
    else if(md.turn==s_left)
    { // turn slight left
        MotorControl.setServo(S_LEFT);
    }
    else if(md.turn==straight)
    { // keep straight
        MotorControl.setServo(STRAIGHT);
    }
    else if(md.turn==s_right)
    { // turn slight right
        MotorControl.setServo(S_RIGHT);
    }
    else if(md.turn==right)
    { // turn hard right
        MotorControl.setServo(HARD_RIGHT);
    }
#endif

#if DC_ON
    //--------------------------- Speed of DC Motor ------------------------
    LD.setNumber(white_mark_count); // Show white_mark_count on Segment display for speed feedback

    if (md.speed == stop)
    { // Stop motor
        speed_factor = dc_stop;
        desired_count = 0;
    }
    else if (md.speed == slow)
    { // Slow speed
        speed_factor = dc_slow;
        desired_count = dc_slow_count;

    }
    else if (md.speed == normal)
    { // Normal Speed
        speed_factor = dc_normal;
        desired_count = dc_normal_count;

    }
    else if (md.speed == turbo)
    { // Turbo Speed
        speed_factor = dc_turbo;
        desired_count = dc_turbo_count;
    }

/////////////////////////////////////////////Speed Encoder////////////////////////////////////////////////////////
    // XXX: Create a "bypass" logic to immediately slow down if needed
#if SPEED_FEEDBACK
    if (check_time > 1000)
    {
        check_time = 0;
        if(white_mark_count==0){
            dead_count++;
        }
        else
            dead_count = 0;

        if(dead_count<RPM_DEAD_SECOND_COUNT)
        {
            switch ((int) md.speed)
            {

                case stop:
                    // Do nothing
                    break;

                case slow:
                    if (desired_count > white_mark_count)
                    { // If car's speed is less than desired slow speed
                        increase_speed(speed_factor,dc_slow);
                    }
                    else if (desired_count < white_mark_count)
                    { // If car's speed is greater than desired slow speed
                        decrease_speed(speed_factor,dc_slow);
                    }
                    break;

                case normal:
                    if (desired_count > white_mark_count)
                    { // If car's speed is less than desired normal speed
                        increase_speed(speed_factor,dc_normal);
                    }
                    else if (desired_count < white_mark_count)
                    { // If car's speed is greater than desired normal speed
                        decrease_speed(speed_factor,dc_normal);
                    }
                    break;

                case turbo:
                    if (desired_count > white_mark_count)
                    { // If car's speed is less than desired turbo speed
                        increase_speed(speed_factor,dc_turbo);
                    }
                    else if (desired_count < white_mark_count)
                    { // If car's speed is greater than desired turbo speed
                        decrease_speed(speed_factor,dc_turbo);
                    }
                    break;
            }
        }
        white_mark_count = 0;
    }
#endif
//////////////////////////////////////////////////////////////////////////////////////////////////////////

    if (check_validity_speed_factor(speed_factor, (int) md.speed) && dead_count<RPM_DEAD_SECOND_COUNT)
    {
        MotorControl.setDC(speed_factor);
    }
    else
    {
        MotorControl.setDC(DC_STOP);
        LE.toggle(SPEED_FACTOR_ERROR);
    }

#endif

    delay_ms(20); // Don't make delay less otherwise you'll have problem.
                  // Required this much delay to set proper pwm for motors
}

void drive_TopGun(void)
{
    set_motors_pwm();
}

void check_rpm(void)
{
    uint16_t result = 0;
    // pick_up_the_last_converted()
    xQueueReceive(g_adc_result_queue, &result, 0);      // Don't wait to get result from g_adc_result_queue

    // XXX: When white mark will be detected white_mark_count gets incremented by 1 and will set white_mark flag will
    //      be true so white_mark_count will not incremented again when sensor will be still on white_mark and when sensor
    //      will detect black_mark then white_mark flag will be false again so if white_mark will come again then
    //      white_mark_count will get incremented...

    if (result > light_threshold && !white_mark)    // Check if white_mark is there and if there then make white_mark flag true and increment white_mark_count
    {
        white_mark_count += 1;
        white_mark = true;
    }
    else if (result < light_threshold && white_mark)    // Check if white_mark flag is true and sensor detects black portion of wheel then set white_mark flag again to false
    {
        white_mark = false;
    }
    check_time += 1;                                // Increment check_time by 1 to
    // start_conversion()
    my_adc0_start_conversion(LIGHT_SENSOR_CHANNEL);
}

bool check_validity_speed_factor(float s_factor, int speed)
{
    bool valid = false;
    switch (speed)
    {
        case slow:
            if (s_factor < DC_THRESH_SLOW)
                valid = false;
            else
                valid = true;
            break;

        case normal:
            if (s_factor < DC_THRESH_NORMAL)
                valid = false;
            else
                valid = true;
            break;

        case turbo:
            if (s_factor < DC_THRESH_TURBO)
                valid = false;
            else
                valid = true;
            break;

        case stop:
            if(s_factor == DC_STOP)
                valid = true;
            else
                valid = false;
            break;
    }
    return valid;
}

void decrease_speed(float& sf,float& ds) {              // sf - speed_factor , ds - dc_speed
    int difference = white_mark_count - desired_count;
    if (difference < 3)
    {
        sf += SPEED_VAR_FACTOR;
        ds += SPEED_VAR_FACTOR;
    }
    else if (difference < 5)
    {
        sf += SPEED_VAR_FACTOR + 0.1;
        ds += SPEED_VAR_FACTOR + 0.1;
    }
    else
    {
        sf += SPEED_VAR_FACTOR + (difference*1.0/60);
        ds += SPEED_VAR_FACTOR + (difference*1.0/60);
    }
}

void increase_speed(float& sf,float& ds) {              // sf - speed_factor , ds - dc_speed
    int difference = desired_count - white_mark_count;
    if (difference < 3)
    {
        sf -= SPEED_VAR_FACTOR;
        ds -= SPEED_VAR_FACTOR;
    }
    else if (difference < 5)
    {
        sf -= SPEED_VAR_FACTOR + 0.1;
        ds-= SPEED_VAR_FACTOR + 0.1;
    }
    else
    {
        sf -= SPEED_VAR_FACTOR + 0.15;
        ds -= SPEED_VAR_FACTOR + 0.15;
    }
}
