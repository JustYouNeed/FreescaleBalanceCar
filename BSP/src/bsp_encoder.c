/**
  *******************************************************************************************************
  * File Name: bsp_encoder.c
  * Author: Vector
  * Version: V1.2.0
  * Date: 2018-2-28
  * Brief: 本文件提供了有关编码器的基本操作函数,如初始化、读取编码器值等,编码器采用中断方式计数
  *******************************************************************************************************
  * History	
  *		1.Author: Vector
	*			Date: 2018-2-28
	*     Mod: 建立文件
	*
	*		2.Author: Vector
	*			Date: 2018-3-2
	*			Mod: 1.将原本独立的编码器数据整合到Car结构体中,便于管理
	*
	*		3.Author: Vector
	*			Date: 2018-3-9
	*			Mod: 1.修复计数器出现溢出时速度计算出现很大负值BUG
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
* Description: 初始化编码器
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
* Description: 由编码器的值计算电机速度
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : 该函数应该定时调用,以提高速度计算精度,同时该函数计算出来的值只能作为参考值,不是很精确
*********************************************************************************************************
*/
extern uint8_t TimerTaskRunMutexSignal;
void bsp_encoder_SpeedCalc(void)
{
	static int32_t LastLeftEncoder, LastRightEncoder;
//	static int32_t LastLeftSpeed, LastRightSpeed;
	static uint32_t LastTime;
	int32_t runtime;

	/*  如果有其他中断函数正在执行,则直接返回  */
	if(TimerTaskRunMutexSignal == 1) return ;
	
	/*  标定定时器函数正在运行  */
	TimerTaskRunMutexSignal = 1;

	/*  读取电机编码器的值  */
	Car.Motor.LeftEncoder = ftm_count_get(ftm0);
	Car.Motor.RightEncoder = ftm_count_get(ftm1);
	
	ftm_count_clean(ftm0);
	ftm_count_clean(ftm1);
	
	/*  计算速度  */
	runtime = bsp_tim_GetRunTime() - LastTime;
	Car.Motor.LeftSpeed = (int32_t)(Car.Motor.LeftEncoder - 0) / 1;
	Car.Motor.RightSpeed = (int32_t)(Car.Motor.RightEncoder - 0) / 1;
	
//	/*  如果速度出现负值说明计数器溢出了,使用上次的速度作为本次的速度  */
//	Car.Motor.LeftSpeed = (Car.Motor.LeftSpeed < 0) ? (LastLeftSpeed) : Car.Motor.LeftSpeed;
//	Car.Motor.RightSpeed = (Car.Motor.RightSpeed < 0) ? (LastRightSpeed) : Car.Motor.RightSpeed;
//	
//	/*  保存上一时刻的速度  */
//	LastLeftSpeed = Car.Motor.LeftSpeed;
//	LastRightSpeed = Car.Motor.RightSpeed;
	
	
	/*  更新时刻,编码器值,为下次计算作准备  */
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


