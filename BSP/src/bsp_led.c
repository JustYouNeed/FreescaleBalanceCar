/**
  *******************************************************************************************************
  * File Name: bsp_led.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-2-17
  * Brief: ���ļ�Ϊ�弶LED������
  *******************************************************************************************************
  * History
	*		1.Author:	Vector
	*			Date:	2018-2-17
  *			Mod:	�����ļ�
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
* Description: ��ʼ��LED����
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
	/*  ��ʼ��LED����Ϊ���,��ʼ��ƽΪ��  */
	gpio_init(LED1_PIN, GPO, 1);
	gpio_init(LED2_PIN, GPO, 1);
	gpio_init(LED3_PIN, GPO, 1);
	bsp_led_OFF(LED_ALL);
}


/*
*********************************************************************************************************
*                                          bsp_led_ON
*
* Description: ��һ��LED��
*             
* Arguments  : 1> LEDx:	Ҫ�򿪵�LED�Ʊ��
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
			gpio_set(LED1_PIN, LED_ON);
			gpio_set(LED2_PIN, LED_ON);
		}break;
		case LED1: gpio_set(LED1_PIN, LED_ON);break;
		case LED2: gpio_set(LED2_PIN, LED_ON);break;
		default: break;
	}
}

/*
*********************************************************************************************************
*                                          bsp_led_OFF
*
* Description: �ر�һ��LED��
*             
* Arguments  : 1> LEDx:	Ҫ�رյ�LED�Ʊ��
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
			gpio_set(LED1_PIN, LED_OFF);
			gpio_set(LED2_PIN, LED_OFF);
		}break;
		case LED1: gpio_set(LED1_PIN, LED_OFF);break;
		case LED2: gpio_set(LED2_PIN, LED_OFF);break;
		default: break;
	}
}


/*
*********************************************************************************************************
*                                       bsp_led_Toggle   
*
* Description: �л�LED�Ƶ�״̬
*             
* Arguments  : 1> LEDx:	Ҫ�л���LED�Ʊ��
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
			gpio_turn(LED1_PIN);
			gpio_turn(LED2_PIN);
		}break;
		case LED1: gpio_turn(LED1_PIN);break;
		case LED2: gpio_turn(LED2_PIN);break;
		default: break;
	}
}


/********************************************  END OF FILE  *******************************************/

