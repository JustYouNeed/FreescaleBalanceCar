/**
  *******************************************************************************************************
  * File Name: bsp_switch.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-3-2
  * Brief: ���ļ��ṩ�������ϰ��뿪�صĻ�����������
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-2
	*			Mod: �����ļ�
  *
  *******************************************************************************************************
  */	
	

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/

# include "bsp_switch.h"

/*
*********************************************************************************************************
*                     bsp_switch_Config                     
*
* Description: ��ʼ�������ϵİ��뿪��
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_switch_Config(void)
{
	/*  ��ʼ�����뿪������Ϊ����  */
	gpio_init(BIT1_PIN, GPI,1);
	gpio_init(BIT2_PIN, GPI,1);
	gpio_init(BIT3_PIN, GPI,1);
	gpio_init(BIT4_PIN, GPI,1);
	
	/*  ���ڰ��뿪�عرպ�Ϊ�͵�ƽ,����ȿ�����������  */
	port_pull(BIT1_PIN);
	port_pull(BIT2_PIN);
	port_pull(BIT3_PIN);
	port_pull(BIT4_PIN);
}


/*
*********************************************************************************************************
*                         bsp_switch_GetValue                 
*
* Description: ��ȡ���뿪�ص�״̬
*             
* Arguments  : None.
*
* Reutrn     : 1> ���뿪�ص�״̬
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint8_t bsp_switch_GetValue(void)
{
	uint8_t value = 0;
	
//	value |= (gpio_get(BIT1_PIN) == 1) ? (1 << 0 ) : (value);
//	value |= (gpio_get(BIT2_PIN) == 1) ? (1 << 1) : (value);
//	value |= (gpio_get(BIT3_PIN) == 1) ? (1 << 2) : (value);
//	value |= (gpio_get(BIT4_PIN) == 1) ? (1 << 3) : (value);
	value |= (uint8_t)((gpio_get(BIT1_PIN) << 0) | (gpio_get(BIT2_PIN) << 1));
	value |= (uint8_t)((gpio_get(BIT3_PIN) << 2) | (gpio_get(BIT4_PIN) << 3));
	
	return value;
}

/********************************************  END OF FILE  *******************************************/
