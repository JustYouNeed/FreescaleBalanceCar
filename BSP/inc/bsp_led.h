/**
  *******************************************************************************************************
  * File Name: bsp_led.c
  * Author: Vector
  * Version: V1.0.1
  * Date: 2018-2-25
  * Brief: 本文件声明了有关板级LED灯的变量声明
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date:	2018-2-25
	*			Mod:	建立文件 
	*
	*		2.Author:
	*			Date: 2018-3-1
	*			Mod: LED灯ID的定义由宏定义改为枚举变量
  *
  *******************************************************************************************************
  */	
# ifndef __BSP_LED_H
# define __BSP_LED_H

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "headfile.h"

/*  定义按键开关时引脚的状态  */
# define LED_OFF				0
# define LED_ON					1

/*  LED灯引脚定义  */
# define LED_RED_PIN			B0
# define LED_BLUE_PIN			B1

/*  LED ID枚举变量  */
typedef enum
{
	LED_ALL = 0x0,
	LED_RED,
	LED_BLUE,
}LED_IDType;


void bsp_led_Config(void);
void bsp_led_ON(uint8_t LEDx);
void bsp_led_OFF(uint8_t LEDx);
void bsp_led_Toggle(uint8_t LEDx);

# endif

/********************************************  END OF FILE  *******************************************/

