# include "FreescaleCar.h"
# include "app_debug.h"
# include "bsp_mpu.h"
# include "display.h"

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
	uint8_t key = KEY_NONE;
	/*  初始化基本使用到的器件,如LED,OLED,按键,MPU等  */
	bsp_Config();
	
	/*  初始化小车参数  */
	Car_ParaInit();
	
	displayInit();
	setShow_ui(MAIN_UI);
	
		/*  选择是否需要校准传感器  */
	if(gpio_get(KEY_OK_PIN) == 0)
		bsp_sensor_Calibration();
	
	/*  创建一个软件定时器,周期49ms,用于上传小车数据到上位机  */
	bsp_tim_CreateSoftTimer(0, 49, debug_CarDataReport, TIMER_MODE_AUTO);
	
	bsp_tim_CreateSoftTimer(1, 20, bsp_key_Scan, TIMER_MODE_AUTO);
	
	/*  创建一个软件定时器,周期333ms,用于指示小车运行状态  */
	bsp_tim_CreateSoftTimer(2, 333, Car_Running, TIMER_MODE_AUTO);
	
	bsp_tim_CreateSoftTimer(3, 100, displayTask, TIMER_MODE_AUTO);
	
	/*  创建一个硬件定时器,周期3ms,用于小车控制  */
	bsp_tim_CreateHardTimer(1, 1, Car_Control);
	bsp_tim_CreateHardTimer(0,5, bsp_mpu_GetAngle);
//	bsp_motor_SetPwm(200, 200);
	while(1)
	{	
//		if(mpu_dmp_get_data(&Car.MPU.Pitch, &Car.MPU.Roll, &Car.MPU.Yaw) == 0)
//		{
////			bsp_mpu_ReadGyro(&Car.MPU.Gyrox, &Car.MPU.Gyroy, &Car.MPU.Gyroz);
////			bsp_mpu_GetAngle();
//		}
	}
}

