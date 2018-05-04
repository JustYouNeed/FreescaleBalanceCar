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

/*  �������  */
# define MOTOR_DEAD_VAL 10

/*  ������ÿȦ���������  */
# define ENCONDER_LINES			512
# define ENCONDER_TEETH			29
# define WHEEL_TEETH				68

/*  �����ܳ�,��λ ��  */
# define WHEEL_GIRTH				0.2

/*  �ٶ�ת����������,������ɺ��ٶȵ�λΪ m/s  */
# define CAR_SPEED_CONSTANT	(1000.0/SPEED_CONTROL_PERIOD/ENCONDER_LINES)//*ENCONDER_TEETH/WHEEL_TEETH * WHEEL_GIRTH



/*  ������������ṹ�嶨��  */
typedef struct
{
	PID_TypeDef PID;											/*  PID����  */
	Sensor_TypeDef Sensor[SENSOR_COUNT];	/*  ������  */
	Motor_TypeDef Motor;									/*  ���  */
	MPU_TypeDef MPU;											/*  MPU����  */
	FuzzyPID_TypeDef DirFuzzy;						/*  ת�����ģ��PID  */

	float HorizontalAE, VecticalAE;				/*  ������ˮƽ����ֱ�Ͳ��  */
	float BalanceAngle;										/*  �޵���ʱվ���ĽǶ�  */
	float CarSpeed, TargetSpeed;	/*  ��ǰ����,����Ŀ���ٶ�,������Ŀ���ٶ�  */
	int16_t MaxPWM;
}Car_TypeDef;

extern Car_TypeDef Car;

void Car_ParaInit(void);
void Car_Control(void);
void Car_ParaStroe(void);
void Car_Running(void);
void Car_ControlStop(void);
void Car_Reset(void);
void Car_ControlStart(void);
void Car_ParaStore(void);

# endif

/********************************************  END OF FILE  *******************************************/


