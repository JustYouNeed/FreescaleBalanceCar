/**
  *******************************************************************************************************
  * File Name: bsp_motor.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-3-1
  * Brief: ���ļ��ṩ�˳��ӵ�����ƵĻ�������
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
# include "bsp_motor.h"
# include "FreescaleCar.h"

/*
*********************************************************************************************************
*                           bsp_motor_Config               
*
* Description: ��ʼ�������������.PWM
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
* Description: ���õ��PWM
*             
* Arguments  : 1> LeftPwm: ��ߵ��PWM
*							 2> RightPwm: �ұߵ��PWM
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_motor_SetPwm(int16_t LeftPwm, int16_t RightPwm)
{
	/*  �������PWM  */
	if(RightPwm > 0)	/*  �����ת  */
	{
		ftm_pwm_duty(ftm2, DRV_PWM1_CHANNEL, RightPwm);
		ftm_pwm_duty(ftm2, DRV_PWM2_CHANNEL, 0);
	}
	else		/*  ��ת  */
	{
		ftm_pwm_duty(ftm2, DRV_PWM1_CHANNEL, 0);
		ftm_pwm_duty(ftm2, DRV_PWM2_CHANNEL, -RightPwm);
	}
	
	/*  �������PWM  */
	if(LeftPwm > 0)	/*  �����ת  */
	{
		ftm_pwm_duty(ftm2, DRV_PWM4_CHANNEL, LeftPwm);
		ftm_pwm_duty(ftm2, DRV_PWM3_CHANNEL, 0);
	}
	else		/*  ��ת  */
	{
		ftm_pwm_duty(ftm2, DRV_PWM4_CHANNEL, 0);
		ftm_pwm_duty(ftm2, DRV_PWM3_CHANNEL, -LeftPwm);
	}
}

/*
*********************************************************************************************************
*                          bsp_motor_Stop                
*
* Description: ֹͣ���
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
//	DRV_DISABLE();	/*  �ر�����  */
	
	/*  PWM����ռ�ձ�Ϊ0  */
	ftm_pwm_duty(ftm2, DRV_PWM1_CHANNEL, 0);
	ftm_pwm_duty(ftm2, DRV_PWM2_CHANNEL, 0);
	ftm_pwm_duty(ftm2, DRV_PWM3_CHANNEL, 0);
	ftm_pwm_duty(ftm2, DRV_PWM4_CHANNEL, 0);
}
	
	

/********************************************  END OF FILE  *******************************************/

