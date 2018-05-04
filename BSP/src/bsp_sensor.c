/**
  *******************************************************************************************************
  * File Name: bsp_sensor
  * Author: Vector
  * Version: V1.1.0
  * Date: 2018-3-1
  * Brief: 本文件为电磁传感器提供了基本的操作函数
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-1
	*			Mod: 建立文件
	*
	*		1.Author: Vector
	*			Date: 2018-3-2
	*			Mod: 1.添加新函数bsp_sensor_DataNormalized,bsp_sensor_DataCopy,bsp_sensor_Calibration
	*					 2.将独立的传感器数据整合到由Car结构体中,便于管理
  *
  *******************************************************************************************************
  */	

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "bsp_sensor.h"
# include "app_sort.h"
# include "drv_adc.h"
# include "app_filter.h"
# include "FreescaleCar.h"

extern uint16_t ADC_Value[SENSOR_COUNT];


/*
*********************************************************************************************************
*                         bsp_sensor_Config                 
*
* Description: 初始化电磁传感器
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_sensor_Config(void)
{	
	adc_init(ADC0_SE8);		/*  SENSOR_V_RM  */
	adc_init(ADC0_SE9);		/*  SENSOR_H_RM  */
	adc_init(ADC0_SE10);		/*  SENSOR_H_R  */
	adc_init(ADC0_SE11);		/*  SENSOR_H_L  */
	
	adc_init(ADC0_SE14);	/*  SENSOR_H_LM  */
	adc_init(ADC0_SE15);	/*  SENSOR_V_LM  */
}

/*
*********************************************************************************************************
*                                bsp_sensor_DataNormalized          
*
* Description: 传感器数据归一化处理
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : 该函数为本文件私有函数
*********************************************************************************************************
*/
void bsp_sensor_DataNormalized(void)
{
	uint8_t cnt = 0;
	
	/*  循环归一化每一个传感器的参数  */
	for(; cnt < SENSOR_COUNT; cnt ++)
	{
		Car.Sensor[cnt].NormalizedValue = (float)(Car.Sensor[cnt].Average - Car.Sensor[cnt].CalibrationMin) / 
																						 (Car.Sensor[cnt].CalibrationMax - Car.Sensor[cnt].CalibrationMin);
	}
}


/*
*********************************************************************************************************
*                            bsp_sensor_DataCopy              
*
* Description: 数据拷贝函数
*             
* Arguments  : 1> dst: 目标缓存区
*              2> src: 数据源地址
*              3> length: 要拷贝的数据长度
*
* Reutrn     : None.
*
* Note(s)    : 目标地址的大小不可以小于要拷贝的数据长度
*********************************************************************************************************
*/
void bsp_sensor_DataCopy(uint16_t *dst, uint16_t *src, uint16_t length)
{
	while(length--) *dst++ = *src++;
}


/*
*********************************************************************************************************
*                           bsp_sensor_DataProcess               
*
* Description: 传感器数据处理函数
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : 该函数应该周期性调用
*********************************************************************************************************
*/
void bsp_sensor_DataProcess(void)
{	
	uint8_t cnt = 0;
	
	uint8_t adc = 0;
	/*  循环处理每一个传感器的值  */
	for(; cnt < SENSOR_COUNT; cnt ++)
	{
		switch(cnt)
		{
			case SENSOR_H_L: adc = adc_once(SENSOR_5, ADC_8bit);break;
			case SENSOR_H_R: adc = adc_once(SENSOR_6, ADC_8bit);break;
		}
		Car.Sensor[cnt].FIFO[Car.Sensor[cnt].Write++] = adc;		/*  写入FIFO  */
		
		if(Car.Sensor[cnt].Write >= SENSOR_FIFO_SIZE) Car.Sensor[cnt].Write = 0;	/*  环形队列  */
		
		filter_SildingAverage(Car.Sensor[cnt].FIFO, &Car.Sensor[cnt].Average, SENSOR_FIFO_SIZE);	/*  滑动平均滤波器  */
		bsp_sensor_DataNormalized();		/*  归一化  */
	}
	
	/*  计算水平差比和  */
	Car.HorizontalAE = 100 * (float)(Car.Sensor[SENSOR_H_L].NormalizedValue - Car.Sensor[SENSOR_H_R].NormalizedValue) / 
														(Car.Sensor[SENSOR_H_L].NormalizedValue + Car.Sensor[SENSOR_H_R].NormalizedValue);
	/*  垂直差比和  */
//	Car.VecticalAE = ((Car.Sensor[SENSOR_V_LM].NormalizedValue - Car.Sensor[SENSOR_H_RM].NormalizedValue) / 
//													(Car.Sensor[SENSOR_V_LM].NormalizedValue + Car.Sensor[SENSOR_H_RM].NormalizedValue));
}

/*
*********************************************************************************************************
*                       bsp_sensor_Calibration                   
*
* Description: 传感器数据校准函数,用于标志跑道上各传感器的最大最小值
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_sensor_Calibration(void)
{
	uint16_t i = 0, j;
	
//	uint16_t ADC_ValueTemp[SENSOR_COUNT];
	uint16_t CalibrationValueTemp[SENSOR_COUNT * 2] = {0};
	
	bsp_led_ON(LED_RED);
//	bsp_oled_ShowString(0, 0, (uint8_t *)"Calibration...");
	
	/*  先假设最大最小值都为0  */
	for(j = 0; j < SENSOR_COUNT; j++)
	{
		Car.Sensor[j].CalibrationMax = 0;
		Car.Sensor[j].CalibrationMin = 0;
	}
	
	/*  平行移动车子,找出传感器的最大最小值  */
	for(; i < 3000; i ++)
	{		
		/*  循环处理每一个传感器  */
		for(j = 0; j < SENSOR_COUNT; j ++)
		{
				switch(j)
			{

				case SENSOR_H_L: Car.Sensor[SENSOR_H_L].FIFO[Car.Sensor[SENSOR_H_L].Write++] = adc_once(SENSOR_5, ADC_8bit);break;
				case SENSOR_H_R: Car.Sensor[SENSOR_H_R].FIFO[Car.Sensor[SENSOR_H_R].Write++] = adc_once(SENSOR_6, ADC_8bit);break;
			}
			if(Car.Sensor[j].Write >= SENSOR_FIFO_SIZE) Car.Sensor[j].Write = 0;	/*  环形FIFO  */
			
			sort_QuickSort(Car.Sensor[j].FIFO, 0, SENSOR_FIFO_SIZE-1);	/*  对数据进行排序,由小到大  */
			
			/*  保存最大值  */
			if(Car.Sensor[j].CalibrationMax < Car.Sensor[j].FIFO[SENSOR_FIFO_SIZE - 1])		
				Car.Sensor[j].CalibrationMax = Car.Sensor[j].FIFO[SENSOR_FIFO_SIZE - 1];
			
			/*  找到最小值  */
			if(Car.Sensor[j].CalibrationMin > Car.Sensor[j].FIFO[0])
				Car.Sensor[j].CalibrationMin = Car.Sensor[j].FIFO[0];
		}
		bsp_led_Toggle(1);
		bsp_tim_DelayMs(5);
	}
	
	/*  先将各标定值暂存到缓存区,便于写入Flash  */
	for(j = 0; j < SENSOR_COUNT; j ++)
	{
		CalibrationValueTemp[j] = Car.Sensor[j].CalibrationMax;
		CalibrationValueTemp[j + SENSOR_COUNT] = Car.Sensor[j + SENSOR_COUNT].CalibrationMin;
	}
	bsp_led_OFF(LED_RED);
//	bsp_oled_ShowString(0, 2, (uint8_t *)"Calibration OK!");
//	bsp_tim_DelayMs(500);
	/*  保存标定最大值到FLASH  */
	FLASH_EraseSector(SENSOR_PARA_FLASH_ADDR);
	FLASH_WriteSector(SENSOR_PARA_FLASH_ADDR, (const uint8_t *)CalibrationValueTemp, SENSOR_COUNT * 4, 0);
}

/********************************************  END OF FILE  *******************************************/



