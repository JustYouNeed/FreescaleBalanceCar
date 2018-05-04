/**
  *******************************************************************************************************
  * File Name: bsp_led.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-2-17
  * Brief: 本文件为板级LED灯驱动
  *******************************************************************************************************
  * History
	*		1.Author:	Vector
	*			Date:	2018-2-17
  *			Mod:	建立文件
  *
  *******************************************************************************************************
  */	


/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "bsp_led.h"


/*
*********************************************************************************************************
*                                   bsp_led_Config       
*
* Description: 初始化LED引脚
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_led_Config(void)
{
	gpio_init(LED_RED_PIN, GPO, LED_OFF);
	gpio_init(LED_BLUE_PIN, GPO, LED_OFF);
}


/*
*********************************************************************************************************
*                                          bsp_led_ON
*
* Description: 打开一个LED灯
*             
* Arguments  : 1> LEDx:	要打开的LED灯编号
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_led_ON(uint8_t LEDx)
{
	switch(LEDx)
	{
		case LED_ALL: 
		{
			gpio_set(LED_RED_PIN, LED_ON);
			gpio_set(LED_BLUE_PIN, LED_ON);
		}break;
		case LED_RED: gpio_set(LED_RED_PIN, LED_ON);break;
		case LED_BLUE: gpio_set(LED_BLUE_PIN, LED_ON);break;
		default: break;
	}
}

/*
*********************************************************************************************************
*                                          bsp_led_OFF
*
* Description: 关闭一个LED灯
*             
* Arguments  : 1> LEDx:	要关闭的LED灯编号
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_led_OFF(uint8_t LEDx)
{
	switch(LEDx)
	{
		case LED_ALL: 
		{
			gpio_set(LED_RED_PIN, LED_OFF);
			gpio_set(LED_BLUE_PIN, LED_OFF);
		}break;
		case LED_RED: gpio_set(LED_RED_PIN, LED_OFF);break;
		case LED_BLUE: gpio_set(LED_BLUE_PIN, LED_OFF);break;
		default: break;
	}
}


/*
*********************************************************************************************************
*                                       bsp_led_Toggle   
*
* Description: 切换LED灯的状态
*             
* Arguments  : 1> LEDx:	要切换的LED灯编号
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_led_Toggle(uint8_t LEDx)
{
	switch(LEDx)
	{
		case LED_ALL: 
		{
			gpio_turn(LED_RED_PIN);
			gpio_turn(LED_BLUE_PIN);
		}break;
		case LED_RED: gpio_turn(LED_RED_PIN);break;
		case LED_BLUE: gpio_turn(LED_BLUE_PIN);break;
		default: break;
	}
}


/********************************************  END OF FILE  *******************************************/

