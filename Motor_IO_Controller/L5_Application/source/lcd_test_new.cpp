/*
 * lcd_test_new.cpp
 *
 *  Created on: Nov 1, 2015
 *      Author: ANUJKORAT
 */

#include "lcd_test_new.hpp"
#include "io.hpp"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "utilities.h"
#include "_can_dbc/generated_motorio_code.h"

bool flag_mag = false;
bool flag_mag_page = false;
bool flag_home = false;
bool flag_home_page = false;
char lcd_char;

lcd_screen_t lcdscreen = home;      // Enum variable for current screen on LCD
extern GEO_TX_GEO_SPEED_ANGLE_t geo_msg;
extern DRIVER_TX_MOTORIO_DIRECTION_t motor_msg;

char Bytes6[6]={0};                 // 6 bytes received from the LCD
char ack=0;                         // Acknowledge byte from LCD



void flag_change(bool* flagp){
    flag_mag = false;
    flag_home = false;
    *flagp = true;
}

void flag_page_change(bool* flagp){
    flag_mag_page = false;
    flag_home_page = false;
    *flagp = true;
}

void lcd_init(void) {
    U3.init(9600,100,100);
}

void lcd_print(){

     if(Bytes6[1]==0x06 && Bytes6[2]==0x00) {
         lcdscreen=Magnetometer;
         //flag_mag = true;
         //flag_home = false;
         flag_change(&flag_mag);
     }
     if(Bytes6[1]==0x1e && Bytes6[2]==0x02) {
         lcdscreen=home;
         //flag_mag = false;
         //flag_home = true;
         flag_change(&flag_home);
     }

     if(flag_mag && !flag_mag_page) {
         put_comm(0x01,0x0a,0x02,0x00,0x00); // switch to form2
         //flag_mag_page = true;
         //flag_home_page = false;
         flag_page_change(&flag_mag_page);
     }
     else if(flag_home && !flag_home_page) {
         put_comm(0x01,0x0a,0x00,0x00,0x00);// switch to form0
         //flag_home_page = true;
         //flag_mag_page = false;
         flag_page_change(&flag_home_page);
     }

     if(flag_mag){
         static char string[30]={0};
         sprintf(string," %x     %x ", motor_msg.MOTORIO_DIRECTION_speed_cmd , motor_msg.MOTORIO_DIRECTION_turn_cmd);
         put_string(string,(uint8_t)strlen(string));
     }

}

void put_comm(char a,char b,char c, char d,char e) {
    char comm[6]={a,b,c,d,e,0};
    for(int i=0;i<5;i++) {
        comm[5]^=comm[i];
    }

    for(int i=0;i<6;i++) {
        U3.putChar(comm[i],10);
    }
    U3.getChar(&ack,10);
}

void put_string(char array[], uint8_t num){
    uint8_t checksum=0x02^0x00^num;
    U3.putChar(0x02,10);
    U3.putChar(0x00,10);
    U3.putChar(num,10);

    for(int i=0;i<num;i++){
        U3.putChar(array[i],10);
        checksum^=array[i];
    }

    U3.putChar(checksum,10);
    U3.getChar(&ack,10);
}


void lcd_receive(){

      U3.getChar(&lcd_char,0);
      if (lcd_char == 0x07)
      {
          Bytes6[0]=lcd_char;
          for(int i=1;i<6;i++) {
              U3.getChar(Bytes6+i,0);
          }
      }
}




