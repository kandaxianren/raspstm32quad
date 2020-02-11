#ifndef _includes_H
#define _includes_H

//#include "includes.h"


#include "stm32f10x.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "wdg.h"
#include "led.h"
#include "pit.h"

#include "spi.h"
#include "24l01.h"
#include "RC.h"
#include "ad.h"
#include <math.h>
#define thro_max 530//430
#define yaw_range 120
#define yaw_range_zuo 80
#define pitch_range 200
#define roll_range 200
#define rccode 0xaa
#define fly_dat 0xab
#define upcode 0xac
//公用函数声明
float map(float input,float input_min,float input_max,float output_min,float output_max);

#endif 
