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

/*  电机死区  */
# define MOTOR_DEAD_VAL 10

/*  编码器每圈输出脉冲数  */
# define ENCONDER_LINES			512
# define ENCONDER_TEETH			29
# define WHEEL_TEETH				68

/*  车轮周长,单位 米  */
# define WHEEL_GIRTH				0.2

/*  速度转换比例因子,计算完成后速度单位为 m/s  */
# define CAR_SPEED_CONSTANT	(1000.0/SPEED_CONTROL_PERIOD/ENCONDER_LINES)//*ENCONDER_TEETH/WHEEL_TEETH * WHEEL_GIRTH



/*  车子整体参数结构体定义  */
typedef struct
{
	PID_TypeDef PID;											/*  PID参数  */
	Sensor_TypeDef Sensor[SENSOR_COUNT];	/*  传感器  */
	Motor_TypeDef Motor;									/*  电机  */
	MPU_TypeDef MPU;											/*  MPU参数  */
	FuzzyPID_TypeDef DirFuzzy;						/*  转向控制模糊PID  */

	float HorizontalAE, VecticalAE;				/*  传感器水平、垂直和差比  */
	float BalanceAngle;										/*  无调节时站立的角度  */
	float CarSpeed, TargetSpeed;	/*  当前车速,整体目标速度,左右轮目标速度  */
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


