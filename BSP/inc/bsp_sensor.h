/**
  *******************************************************************************************************
  * File Name: bsp_sensor.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-3-1
  * Brief: 本文件为主控传感器数据采集提供操作,主控芯片为KEA128-LFQP80
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-1
	*			Mod: 建立文件
  *
  *******************************************************************************************************
  */	
	
# ifndef __BSP_SENSOR_H
# define __BSP_SENSOR_H

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "headfile.h"


# define SENSOR_1				ADC0_SE13
# define SENSOR_2				ADC0_SE12
# define SENSOR_3				ADC0_SE7
# define SENSOR_4				ADC0_SE6
# define SENSOR_5				ADC0_SE5
# define SENSOR_6				ADC0_SE4

# define SENSOR_FIFO_SIZE		10

/* 传感器ID枚举变量   */
typedef enum
{
	SENSOR_V_LM = 0x0,		/*  Vertical Left Most   */
	SENSOR_H_LM,					/*  Horizental Left Most  */
	SENSOR_H_L,						/*  Horizental Left  */
	SENSOR_H_R,						/*  Horizental Right  */
	SENSOR_H_RM,					/*  Horizental Right Most  */
	SENSOR_V_RM,					/*  Vertical Right Most  */
	SENSOR_COUNT,				
}SENSOR_ID_EnumType;

/*  传感器数据结构体  */
typedef struct 
{
	uint8_t Read;
	uint8_t Write;
	uint16_t Average;
	uint16_t CalibrationMax;
	uint16_t CalibrationMin;
	
	float NormalizedValue;
	uint16_t FIFO[SENSOR_FIFO_SIZE];
}Sensor_TypeDef;


void bsp_sensor_Config(void);
void bsp_sensor_DataProcess(void);
void bsp_sensor_Calibration(void);


# endif

/********************************************  END OF FILE  *******************************************/

