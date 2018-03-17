# include "bsp.h"
# include "bsp_mpu.h"
uint8_t TimerTaskRunMutexSignal = 0;

void bsp_Config(void)
{
	get_clk();
	
	
	
	FLASH_Init();				/*  оƬ�ڲ�Flash��ʼ��  */
	bsp_tim_SoftConfig();		/*  ������ʱ����ʼ��,���õδ�ʱ��,��ʱ����1ms  */
	bsp_uart_Config();			/*  ���Դ��ڳ�ʼ��,������115200  */
	bsp_oled_Config();			/*  OLED��Ļ��ʼ��  */
	bsp_led_Config();				/*  LED�Ƴ�ʼ��  */
	bsp_key_Config();				/*  ����������ʼ��  */
	bsp_switch_Config();		/*  ���뿪�س�ʼ��  */
	bsp_encoder_Config();		/*  �����������ʼ��  */
//	DisableInterrupts;
	
	bsp_mpu_Config();				/*  ��ʼ��MPU  */
	
	bsp_motor_Config();			/*  ������Ƴ�ʼ��  */
	bsp_sensor_Config();		/*  ��Ŵ�������ʼ��  */
	EnableInterrupts;						/*  �����ж�  */
}
