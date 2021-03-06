# include "bsp.h"
# include "bsp_mpu.h"
uint8_t TimerTaskRunMutexSignal = 0;

void bsp_Config(void)
{
	get_clk();
	
	EnableInterrupts;						/*  开启中断  */
	FLASH_Init();				/*  芯片内部Flash初始化  */
	bsp_tim_SoftConfig();		/*  软件定时器初始化,采用滴答定时器,定时周期1ms  */
	bsp_uart_Config();			/*  调试串口初始化,波特率115200  */
	bsp_oled_Config();			/*  OLED屏幕初始化  */
	bsp_led_Config();				/*  LED灯初始化  */
	bsp_key_Config();				/*  独立按键初始化  */
	bsp_switch_Config();		/*  拔码开关初始化  */
	bsp_encoder_Config();		/*  电机编码器初始化  */
	
	bsp_mpu_Config();				/*  初始化MPU  */
	
	bsp_motor_Config();			/*  电机控制初始化  */
	bsp_sensor_Config();		/*  电磁传感器初始化  */

}

