/**
  *******************************************************************************************************
  * File Name: app_pid.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-3-2
  * Brief: ���ļ��ṩ��PID�㷨����,�Լ�PID�����Ķ�ȡ���洢����
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
# include "app_pid.h"
# include "FreescaleCar.h"


/*
*********************************************************************************************************
*                        pid_ReadPara                  
*
* Description: ��Flash�ж�ȡ�洢��PID����
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void pid_ReadPara(void)
{
//	Car.PID.Kp_Straight = flash_read(PID_PARA_FLASH_ADDR, 0, float);
//	Car.PID.Ki_Straight = flash_read(PID_PARA_FLASH_ADDR, 4, float);
//	Car.PID.Kd_Straight = flash_read(PID_PARA_FLASH_ADDR, 8, float);
//	
//	Car.PID.Kp_Curved = flash_read(PID_PARA_FLASH_ADDR, 12, float);
//	Car.PID.Ki_Curved = flash_read(PID_PARA_FLASH_ADDR, 16, float);
//	Car.PID.Kd_Curved = flash_read(PID_PARA_FLASH_ADDR, 20, float);
}

/*
*********************************************************************************************************
*                      pid_StorePara                    
*
* Description: ��PID�����洢��Flash��
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void pid_StorePara(void)
{	
	FLASH_EraseSector(PID_PARA_FLASH_ADDR);	/*  �Ȳ���һ��,��Ȼ�޷�д��  */
	FLASH_WriteSector(PID_PARA_FLASH_ADDR, (const uint8_t*)&Car.PID, 56, 0);
}

/********************************************  END OF FILE  *******************************************/

