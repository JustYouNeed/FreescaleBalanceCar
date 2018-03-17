# include "FreescaleCar.h"
# include "app_debug.h"
# include "bsp_mpu.h"

/*
*********************************************************************************************************
*                           main               
*
* Description: C�����־������
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
int main(void)
{	
	/*  ��ʼ������ʹ�õ�������,��LED,OLED,����,MPU��  */
	bsp_Config();
	
	/*  ��ʼ��С������  */
	Car_ParaInit();
	
	/*  ����һ�������ʱ��,����49ms,�����ϴ�С�����ݵ���λ��  */
	bsp_tim_CreateSoftTimer(0, 49, debug_CarDataReport, TIMER_MODE_AUTO);
	
//	/*  ����һ�������ʱ��,����199ms,������ʾС������  */
//	bsp_tim_CreateSoftTimer(1, 301, debug_ShowPara, TIMER_MODE_AUTO);
	
	/*  ����һ�������ʱ��,����333ms,����ָʾС������״̬  */
	bsp_tim_CreateSoftTimer(2, 333, Car_Running, TIMER_MODE_AUTO);
	
//	ADC->SC1 |= 1 << 6;
//		NVIC_EnableIRQ(ADC_IRQn);
//	NVIC_SetPriority(ADC_IRQn, 1);
	/*  ����һ��Ӳ����ʱ��,����3ms,����С������  */
	bsp_tim_CreateHardTimer(0, 10, Car_Control);
	
//	bsp_motor_SetPwm(400, 400);
	while(1)
	{
		if(mpu_dmp_get_data(&Car.MPU.Pitch, &Car.MPU.Roll, &Car.MPU.Yaw) == 0)
		{
//			bsp_mpu_ReadAcc(&Car.MPU.Accx, &Car.MPU.Accy, &Car.MPU.Accz);
			bsp_mpu_ReadGyro(&Car.MPU.Gryox, &Car.MPU.Gryoy, &Car.MPU.Gryoz);
		}
	}
}

