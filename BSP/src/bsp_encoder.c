/**
  *******************************************************************************************************
  * File Name: bsp_encoder.c
  * Author: Vector
  * Version: V1.2.0
  * Date: 2018-2-28
  * Brief: ���ļ��ṩ���йر������Ļ�����������,���ʼ������ȡ������ֵ��,�����������жϷ�ʽ����
  *******************************************************************************************************
  * History	
  *		1.Author: Vector
	*			Date: 2018-2-28
	*     Mod: �����ļ�
	*
	*		2.Author: Vector
	*			Date: 2018-3-2
	*			Mod: 1.��ԭ�������ı������������ϵ�Car�ṹ����,���ڹ���
	*
	*		3.Author: Vector
	*			Date: 2018-3-9
	*			Mod: 1.�޸��������������ʱ�ٶȼ�����ֺܴ�ֵBUG
  *
  *******************************************************************************************************
  */	

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/

# include "bsp_encoder.h"
# include "FreescaleCar.h"
/*
*********************************************************************************************************
*                            bsp_encoder_Config              
*
* Description: ��ʼ��������
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_encoder_Config(void)
{
	ftm_count_init(ftm0);
	ftm_count_init(ftm1);
	
	gpio_init(H5, GPI, 1);
	gpio_init(H7, GPI, 1);
}

/*
*********************************************************************************************************
*                              bsp_encoder_SpeedCalc            
*
* Description: �ɱ�������ֵ�������ٶ�
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : �ú���Ӧ�ö�ʱ����,������ٶȼ��㾫��,ͬʱ�ú������������ֵֻ����Ϊ�ο�ֵ,���Ǻܾ�ȷ
*********************************************************************************************************
*/
extern uint8_t TimerTaskRunMutexSignal;
void bsp_encoder_SpeedCalc(void)
{
	static int32_t LastLeftEncoder, LastRightEncoder;
//	static int32_t LastLeftSpeed, LastRightSpeed;
	static uint32_t LastTime;
	int32_t runtime;

	/*  ����������жϺ�������ִ��,��ֱ�ӷ���  */
	if(TimerTaskRunMutexSignal == 1) return ;
	
	/*  �궨��ʱ��������������  */
	TimerTaskRunMutexSignal = 1;

	/*  ��ȡ�����������ֵ  */
	Car.Motor.LeftEncoder = ftm_count_get(ftm0);
	Car.Motor.RightEncoder = ftm_count_get(ftm1);
	
	ftm_count_clean(ftm0);
	ftm_count_clean(ftm1);
	
	/*  �����ٶ�  */
	runtime = bsp_tim_GetRunTime() - LastTime;
	Car.Motor.LeftSpeed = (int32_t)(Car.Motor.LeftEncoder - 0) / 1;
	Car.Motor.RightSpeed = (int32_t)(Car.Motor.RightEncoder - 0) / 1;
	
//	/*  ����ٶȳ��ָ�ֵ˵�������������,ʹ���ϴε��ٶ���Ϊ���ε��ٶ�  */
//	Car.Motor.LeftSpeed = (Car.Motor.LeftSpeed < 0) ? (LastLeftSpeed) : Car.Motor.LeftSpeed;
//	Car.Motor.RightSpeed = (Car.Motor.RightSpeed < 0) ? (LastRightSpeed) : Car.Motor.RightSpeed;
//	
//	/*  ������һʱ�̵��ٶ�  */
//	LastLeftSpeed = Car.Motor.LeftSpeed;
//	LastRightSpeed = Car.Motor.RightSpeed;
	
	
	/*  ����ʱ��,������ֵ,Ϊ�´μ�����׼��  */
	LastTime = 	bsp_tim_GetRunTime();
//	LastLeftEncoder = Car.Motor.LeftEncoder;
//	LastRightEncoder = Car.Motor.RightEncoder;
	
	if(gpio_get(H5) == 0)
		Car.Motor.LeftSpeed = -Car.Motor.LeftSpeed;
	if(gpio_get(H7) == 1)
		Car.Motor.RightSpeed = -Car.Motor.RightSpeed;
	
	TimerTaskRunMutexSignal = 0;
}

/********************************************  END OF FILE  *******************************************/


