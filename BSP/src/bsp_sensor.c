/**
  *******************************************************************************************************
  * File Name: bsp_sensor
  * Author: Vector
  * Version: V1.1.0
  * Date: 2018-3-1
  * Brief: ���ļ�Ϊ��Ŵ������ṩ�˻����Ĳ�������
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-1
	*			Mod: �����ļ�
	*
	*		1.Author: Vector
	*			Date: 2018-3-2
	*			Mod: 1.�����º���bsp_sensor_DataNormalized,bsp_sensor_DataCopy,bsp_sensor_Calibration
	*					 2.�������Ĵ������������ϵ���Car�ṹ����,���ڹ���
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
* Description: ��ʼ����Ŵ�����
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
//	ADC_InitTypeDef ADC_InitStruct;
//	
//	ADC_InitStruct.ADC_Channel = SENSOR_1 | SENSOR_2 | SENSOR_3 | SENSOR_4 | SENSOR_5 | SENSOR_6;
//	ADC_InitStruct.ADC_ChannelCount = 6;
//	ADC_InitStruct.ADC_ClockSource = ADC_ClockSource_BusClock;
//	ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;
//	ADC_InitStruct.ADC_IRQCmd = ENABLE;
//	ADC_InitStruct.ADC_Prescaler = ADC_Prescaler_Div2;
//	ADC_InitStruct.ADC_RefSource = ADC_RefSource_VDD;
//	ADC_InitStruct.ADC_ScanConvMode = DISABLE;
//	ADC_InitStruct.ADC_RefSource = ADC_Resolution_10b;
//	drv_adc_Init(&ADC_InitStruct);
	
	adc_init(ADC0_SE4);		/*  SENSOR_V_RM  */
	adc_init(ADC0_SE5);		/*  SENSOR_H_RM  */
	adc_init(ADC0_SE6);		/*  SENSOR_H_R  */
	adc_init(ADC0_SE7);		/*  SENSOR_H_L  */
	
	adc_init(ADC0_SE12);	/*  SENSOR_H_LM  */
	adc_init(ADC0_SE13);	/*  SENSOR_V_LM  */
}

/*
*********************************************************************************************************
*                                bsp_sensor_DataNormalized          
*
* Description: ���������ݹ�һ������
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : �ú���Ϊ���ļ�˽�к���
*********************************************************************************************************
*/
void bsp_sensor_DataNormalized(void)
{
	uint8_t cnt = 0;
	
	/*  ѭ����һ��ÿһ���������Ĳ���  */
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
* Description: ���ݿ�������
*             
* Arguments  : 1> dst: Ŀ�껺����
*              2> src: ����Դ��ַ
*              3> length: Ҫ���������ݳ���
*
* Reutrn     : None.
*
* Note(s)    : Ŀ���ַ�Ĵ�С������С��Ҫ���������ݳ���
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
* Description: ���������ݴ�������
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : �ú���Ӧ�������Ե���
*********************************************************************************************************
*/
void bsp_sensor_DataProcess(void)
{	
	uint8_t cnt = 0;
	
//	/*  �����ݴ����������������FIFO,����������,��ΪFIFO����Ҫ��������ƽ���˲�,��������  */
//	uint16_t ADC_ValueTemp[SENSOR_FIFO_SIZE] = {0};	
	
	/*  ���ڴ�ADC�Ĵ����ж�ȡ��ת�����  */
//	uint16_t ADC_Value[SENSOR_COUNT];
	
	/*  ��ȡ�����������ĵ�ѹֵ  */
//	drv_adc_GetMultiADCResult(ADC_Value);
	uint8_t adc = 0;
	/*  ѭ������ÿһ����������ֵ  */
	for(; cnt < SENSOR_COUNT; cnt ++)
	{
		switch(cnt)
		{
			case SENSOR_V_LM: adc = adc_once(SENSOR_1, ADC_8bit);break;
			case SENSOR_H_LM: adc = adc_once(SENSOR_2, ADC_8bit);break;
			case SENSOR_H_L: adc = adc_once(SENSOR_3, ADC_8bit);break;
			case SENSOR_H_R: adc = adc_once(SENSOR_4, ADC_8bit);break;
			case SENSOR_H_RM: adc = adc_once(SENSOR_5, ADC_8bit);break;
			case SENSOR_V_RM: adc = adc_once(SENSOR_6, ADC_8bit);break;
		}
		Car.Sensor[cnt].FIFO[Car.Sensor[cnt].Write++] = adc;		/*  д��FIFO  */
		
		if(Car.Sensor[cnt].Write >= SENSOR_FIFO_SIZE) Car.Sensor[cnt].Write = 0;	/*  ���ζ���  */
		
//		/*  �ݴ�����  */
//		bsp_sensor_DataCopy(ADC_ValueTemp, Car.Sensor[cnt].FIFO, SENSOR_FIFO_SIZE);
//		
//		sort_QuickSort(ADC_ValueTemp, 0, SENSOR_COUNT-1);		/*  ��������,��С����  */
		filter_SildingAverage(Car.Sensor[cnt].FIFO, &Car.Sensor[cnt].Average, SENSOR_FIFO_SIZE);	/*  ����ƽ���˲���  */
		bsp_sensor_DataNormalized();		/*  ��һ��  */
	}
	
	/*  ����ˮƽ��Ⱥ�  */
	Car.HorizontalAE = (float)(Car.Sensor[SENSOR_H_LM].NormalizedValue - Car.Sensor[SENSOR_V_RM].NormalizedValue) / 
														(Car.Sensor[SENSOR_H_LM].NormalizedValue + Car.Sensor[SENSOR_V_RM].NormalizedValue);
	/*  ��ֱ��Ⱥ�  */
	Car.VecticalAE = ((Car.Sensor[SENSOR_V_LM].NormalizedValue - Car.Sensor[SENSOR_H_RM].NormalizedValue) / 
													(Car.Sensor[SENSOR_V_LM].NormalizedValue + Car.Sensor[SENSOR_H_RM].NormalizedValue));
}

/*
*********************************************************************************************************
*                       bsp_sensor_Calibration                   
*
* Description: ����������У׼����,���ڱ�־�ܵ��ϸ��������������Сֵ
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
	
	uint16_t ADC_ValueTemp[SENSOR_COUNT];
	uint16_t CalibrationValueTemp[SENSOR_COUNT * 2] = {0};
	
	bsp_oled_ShowString(0, 0, (uint8_t *)"Calibration...");
	
	/*  �ȼ��������Сֵ��Ϊ0  */
	for(j = 0; j < SENSOR_COUNT; j++)
	{
		Car.Sensor[j].CalibrationMax = 0;
		Car.Sensor[j].CalibrationMin = 0;
	}
	
	/*  ƽ���ƶ�����,�ҳ��������������Сֵ  */
	for(; i < 3000; i ++)
	{
		/*  ��ȡ��������ѹֵ  */
		drv_adc_GetMultiADCResult(ADC_ValueTemp);
		
		/*  ѭ������ÿһ��������  */
		for(j = 0; j < SENSOR_COUNT; j ++)
		{
			Car.Sensor[j].FIFO[Car.Sensor[j].Write++] = ADC_ValueTemp[j];					/*  �����ѹֵ  */
			if(Car.Sensor[j].Write >= SENSOR_FIFO_SIZE) Car.Sensor[j].Write = 0;	/*  ����FIFO  */
			
			sort_QuickSort(Car.Sensor[j].FIFO, 0, SENSOR_FIFO_SIZE-1);	/*  �����ݽ�������,��С����  */
			
			/*  �������ֵ  */
			if(Car.Sensor[j].CalibrationMax < Car.Sensor[j].FIFO[SENSOR_FIFO_SIZE - 1])		
				Car.Sensor[j].CalibrationMax = Car.Sensor[j].FIFO[SENSOR_FIFO_SIZE - 1];
			
			/*  �ҵ���Сֵ  */
			if(Car.Sensor[j].CalibrationMin > Car.Sensor[j].FIFO[0])
				Car.Sensor[j].CalibrationMin = Car.Sensor[j].FIFO[0];
		}
		bsp_tim_DelayMs(5);
	}
	
	/*  �Ƚ����궨ֵ�ݴ浽������,����д��Flash  */
	for(j = 0; j < SENSOR_COUNT; j ++)
	{
		CalibrationValueTemp[j] = Car.Sensor[j].CalibrationMax;
		CalibrationValueTemp[j + SENSOR_COUNT] = Car.Sensor[j + SENSOR_COUNT].CalibrationMin;
	}
	
	bsp_oled_ShowString(0, 2, (uint8_t *)"Calibration OK!");
//	bsp_tim_DelayMs(500);
	/*  ����궨���ֵ��FLASH  */
//	drv_flash_EraseSector(SENSOR_PARA_FLASH_ADDR);
//	drv_flash_WriteSector(SENSOR_PARA_FLASH_ADDR, (const uint8_t *)CalibrationValueTemp, SENSOR_COUNT * 4, 0);
}

/********************************************  END OF FILE  *******************************************/


