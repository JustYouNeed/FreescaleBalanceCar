/**
  *******************************************************************************************************
  * File Name: 
  * Author: 
  * Version: 
  * Date: 
  * Brief: 
  *******************************************************************************************************
  * History
  *
  *
  *******************************************************************************************************
  */	
	
# ifndef __SYSTEM_H
# define __SYSTEM_H

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "bsp.h"
# include "app_pid.h"
# include "bsp_mpu.h"


/*  ������������ṹ�嶨��  */
typedef struct
{
	PID_TypeDef PID;											/*  PID����  */
	Sensor_TypeDef Sensor[SENSOR_COUNT];	/*  ������  */
	Motor_TypeDef Motor;									/*  ���  */
	MPU_TypeDef MPU;											/*  MPU����  */
	
	float HorizontalAE, VecticalAE;				/*  ������ˮƽ����ֱ�Ͳ��  */
	float BalanceAngle;										/*  �޵���ʱվ���ĽǶ�  */
	int16_t OutThreshold[SENSOR_COUNT];		/*  ������ֵ  */
}Car_TypeDef;

extern Car_TypeDef Car;

void Car_ParaInit(void);
void Car_Control(void);
void Car_ParaStroe(void);
void Car_Running(void);


# endif

/********************************************  END OF FILE  *******************************************/


