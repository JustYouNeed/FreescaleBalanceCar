# include "FreescaleCar.h"
# include "app_debug.h"
# include "bsp_mpu.h"

/*
*********************************************************************************************************
*                           main               
*
* Description: C程序标志主函数
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
	/*  初始化基本使用到的器件,如LED,OLED,按键,MPU等  */
	bsp_Config();
	
	/*  初始化小车参数  */
	Car_ParaInit();
	
	/*  创建一个软件定时器,周期49ms,用于上传小车数据到上位机  */
	bsp_tim_CreateSoftTimer(0, 49, debug_CarDataReport, TIMER_MODE_AUTO);
	
//	/*  创建一个软件定时器,周期199ms,用于显示小车数据  */
//	bsp_tim_CreateSoftTimer(1, 301, debug_ShowPara, TIMER_MODE_AUTO);
	
	/*  创建一个软件定时器,周期333ms,用于指示小车运行状态  */
	bsp_tim_CreateSoftTimer(2, 333, Car_Running, TIMER_MODE_AUTO);
	
//	ADC->SC1 |= 1 << 6;
//		NVIC_EnableIRQ(ADC_IRQn);
//	NVIC_SetPriority(ADC_IRQn, 1);
	/*  创建一个硬件定时器,周期3ms,用于小车控制  */
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

