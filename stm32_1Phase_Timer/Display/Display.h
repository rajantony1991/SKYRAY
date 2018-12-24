/*
 * Display.h
 *
 *  Created on: Dec 21, 2018
 *      Author: Skyray power systems
 */

#ifndef DISPLAY_H_
#define DISPLAY_H_

#include "stm32f0xx_hal.h"
#include "stdbool.h"

#define CHAR_A 10
#define CHAR_B 11
#define CHAR_C 12
#define CHAR_D 13
#define CHAR_E 14
#define CHAR_F 15
#define CHAR_G 16
#define CHAR_H 17
#define CHAR_I 18
#define CHAR_J 19
#define CHAR_K 20
#define CHAR_L 21
#define CHAR_M 22
#define CHAR_N 23
#define CHAR_O 24
#define CHAR_P 25
#define CHAR_Q 26
#define CHAR_R 27
#define CHAR_S 28
#define CHAR_T 29
#define CHAR_U 30
#define CHAR_V 31
#define CHAR_W 32
#define CHAR_X 33
#define CHAR_Y 34
#define CHAR_Z 35
#define CHAR__ 36
#define CHAR_n 37
#define CHAR_h 38
#define CAHR_i 39
#define CHAR_a 40
#define Blink_1_Digit      1
#define Blink_2_Digit      2
#define Blink_3_Digit      3
#define Blink_4_Digit      4
#define Blink_ALL_Digit    5
#define Blink_12_Digit     6
#define Blink_34_Digit     7
#define Blink_23_db        8
#define Blink_3_db         9
#define Blink_1HZ          0

void Display_event(uint8_t LED1 , uint8_t LED2 , uint8_t LED3 , uint8_t LED4 , uint8_t Blink);


#endif /* DISPLAY_H_ */
