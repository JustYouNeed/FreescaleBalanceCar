# include "FreescaleCar.h"
# include "app_debug.h"
# include "bsp_mpu.h"
# include "display.h"

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
	uint8_t key = KEY_NONE;
	/*  ��ʼ������ʹ�õ�������,��LED,OLED,����,MPU��  */
	bsp_Config();
	
	/*  ��ʼ��С������  */
	Car_ParaInit();
	
	displayInit();
	setShow_ui(MAIN_UI);
	
		/*  ѡ���Ƿ���ҪУ׼������  */
	if(gpio_get(KEY_OK_PIN) == 0)
		bsp_sensor_Calibration();
	
	/*  ����һ�������ʱ��,����49ms,�����ϴ�С�����ݵ���λ��  */
	bsp_tim_CreateSoftTimer(0, 49, debug_CarDataReport, TIMER_MODE_AUTO);
	
	bsp_tim_CreateSoftTimer(1, 20, bsp_key_Scan, TIMER_MODE_AUTO);
	
	/*  ����һ�������ʱ��,����333ms,����ָʾС������״̬  */
	bsp_tim_CreateSoftTimer(2, 333, Car_Running, TIMER_MODE_AUTO);
	
	bsp_tim_CreateSoftTimer(3, 100, displayTask, TIMER_MODE_AUTO);
	
	/*  ����һ��Ӳ����ʱ��,����3ms,����С������  */
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

