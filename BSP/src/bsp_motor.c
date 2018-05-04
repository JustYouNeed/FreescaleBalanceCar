/**
  *******************************************************************************************************
  * File Name: bsp_motor.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-3-1
  * Brief: 本文件提供了车子电机控制的基本函数
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-2
	*			Mod: 建立文件
  *
  *******************************************************************************************************
  */	
	
/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "bsp_motor.h"
# include "FreescaleCar.h"

/*
*********************************************************************************************************
*                           bsp_motor_Config               
*
* Description: 初始化电机控制引脚.PWM
*             
* Arguments  : None
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_motor_Config(void)
{
	ftm_pwm_init(ftm2, DRV_PWM1_CHANNEL, 15000, 0);
	ftm_pwm_init(ftm2, DRV_PWM2_CHANNEL, 15000, 0);
	ftm_pwm_init(ftm2, DRV_PWM3_CHANNEL, 15000, 0);
	ftm_pwm_init(ftm2, DRV_PWM4_CHANNEL, 15000, 0);
//	NVIC_SetPriority(FTM2_IRQn, 3);
}
extern uint16_t period[3];

/*
*********************************************************************************************************
*                          bsp_motor_SetPwm                
*
* Description: 设置电机PWM
*             
* Arguments  : 1> LeftPwm: 左边电机PWM
*							 2> RightPwm: 右边电机PWM
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_motor_SetPwm(int16_t LeftPwm, int16_t RightPwm)
{
	/*  设置左边PWM  */
	if(RightPwm > 0)	/*  电机正转  */
	{
		ftm_pwm_duty(ftm2, DRV_PWM1_CHANNEL, RightPwm);
		ftm_pwm_duty(ftm2, DRV_PWM2_CHANNEL, 0);
	}
	else		/*  反转  */
	{
		ftm_pwm_duty(ftm2, DRV_PWM1_CHANNEL, 0);
		ftm_pwm_duty(ftm2, DRV_PWM2_CHANNEL, -RightPwm);
	}
	
	/*  设置左边PWM  */
	if(LeftPwm > 0)	/*  电机正转  */
	{
		ftm_pwm_duty(ftm2, DRV_PWM4_CHANNEL, LeftPwm);
		ftm_pwm_duty(ftm2, DRV_PWM3_CHANNEL, 0);
	}
	else		/*  反转  */
	{
		ftm_pwm_duty(ftm2, DRV_PWM4_CHANNEL, 0);
		ftm_pwm_duty(ftm2, DRV_PWM3_CHANNEL, -LeftPwm);
	}
}

/*
*********************************************************************************************************
*                          bsp_motor_Stop                
*
* Description: 停止电机
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_motor_Stop(void)
{
//	DRV_DISABLE();	/*  关闭驱动  */
	
	/*  PWM设置占空比为0  */
	ftm_pwm_duty(ftm2, DRV_PWM1_CHANNEL, 0);
	ftm_pwm_duty(ftm2, DRV_PWM2_CHANNEL, 0);
	ftm_pwm_duty(ftm2, DRV_PWM3_CHANNEL, 0);
	ftm_pwm_duty(ftm2, DRV_PWM4_CHANNEL, 0);
}
	
	

/********************************************  END OF FILE  *******************************************/

