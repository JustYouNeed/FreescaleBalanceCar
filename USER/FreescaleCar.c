/**
  *******************************************************************************************************
  * File Name: FreescaleCar.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-3-3
  * Brief: 本文件用于车子数据记录、处理、保存等，同时车子控制函数也在本文件
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-4
	*			Mod: 建立文件
  *
  *******************************************************************************************************
  */	
	
	
/*
	*******************************************************************************************************
	*                              INCLUDE FILES
	*******************************************************************************************************
*/
# include "FreescaleCar.h"
# include "bsp_mpu.h"

/*  小车控制结构体,控制小车掺需要用到的数据全部在这个结构体里面,且可以直接使用  */
Car_TypeDef Car;


/*
*********************************************************************************************************
*                         Car_ParaInit                 
*
* Description: 车子参数初始化,从芯片Flash中读取出存储的电机参数
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void Car_ParaInit(void)
{
	uint8_t i = 0;
	
	/*  初始化车子的各差比  */
	Car.HorizontalAE = 0;
	Car.VecticalAE = 0;
		
	/*  初始化车子的PID参数,从Flash中读取出保存的PID参数  */
	Car.PID.Balance_Kp = flash_read(PID_PARA_FLASH_ADDR, 0, float);	/*  直立PID参数  */
	Car.PID.Balance_Ki = flash_read(PID_PARA_FLASH_ADDR, 4, float);
	Car.PID.Balance_Kd = flash_read(PID_PARA_FLASH_ADDR, 8, float);
	
	Car.PID.Velocity_Kp = flash_read(PID_PARA_FLASH_ADDR, 12, float);		/*  速度环PID参数  */
	Car.PID.Velocity_Ki = flash_read(PID_PARA_FLASH_ADDR, 16, float);
	Car.PID.Velocity_Kd = flash_read(PID_PARA_FLASH_ADDR, 20, float);
	
	Car.PID.Turn_Kp = flash_read(PID_PARA_FLASH_ADDR, 24, float);		/*  转向环PID参数  */
	Car.PID.Turn_Ki = flash_read(PID_PARA_FLASH_ADDR, 28, float);
	Car.PID.Turn_Kd = flash_read(PID_PARA_FLASH_ADDR, 32, float);
	
	/*  初始化车子的传感器参数,从Flash中读取标定值  */
	for(i = 0; i < SENSOR_COUNT; i ++)
	{
		Car.Sensor[i].FIFO[0] = 0;
		Car.Sensor[i].Read = 0;
		Car.Sensor[i].Write = 0;
		Car.Sensor[i].Average = 0;
		Car.Sensor[i].NormalizedValue = 0.0f;
		Car.Sensor[i].CalibrationMax = flash_read(SENSOR_PARA_FLASH_ADDR, i * 2, uint16_t);
		Car.Sensor[i].CalibrationMin = flash_read(SENSOR_PARA_FLASH_ADDR, (i + SENSOR_COUNT) * 2, uint16_t);
	}
	
	/*  电机控制参数初始化  */
	Car.Motor.PWM_Frequency = 10;	/*  电机PWM频率为10KHz  */
	Car.Motor.LeftPwm = 0;
	Car.Motor.RightPwm = 0;
	Car.Motor.LeftEncoder = 0;
	Car.Motor.RightEncoder = 0;
	Car.Motor.LeftSpeed = 0;
	Car.Motor.RightSpeed = 0;
	
	
	Car.BalanceAngle = 12.4;
	
}

/*
*********************************************************************************************************
*                            Car_ParaStroe              
*
* Description: 
*             
* Arguments  : 
*
* Reutrn     : 
*
* Note(s)    : 
*********************************************************************************************************
*/
void Car_ParaStroe(void)
{
//	FLASH_EraseSector(CAR_PARA_FLASH_ADDR);
//	FLASH_WriteSector(CAR_PARA_FLASH_ADDR, (const uint8_t *)&Car.BaseSpeed, 2, 0);
}


/*
*********************************************************************************************************
*                        Car_Running                  
*
* Description: 小车运行指示,LED灯闪烁
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void Car_Running(void)
{
	bsp_led_Toggle(1);
}


/*
*********************************************************************************************************
*                        Car_BalancePIDCalc                  
*
* Description: 小车直立环PID计算
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
int16_t Car_BalancePIDCalc(float angle, float pitch)
{
	float fBias;   //每个时刻的偏差
	int16_t iBalance_PWM; //计算后的PWM
	static float sfBalance_ErrSum;//	
	
	fBias = angle - Car.BalanceAngle;//偏差等于当前角度减去初始化时的偏差角再减去无电机控制时能直立的角度
	
	Car.PID.ErrSum += fBias; //以和代替积分
	
	if(sfBalance_ErrSum > 7200) sfBalance_ErrSum = 7200;  //积分限幅
	else if(sfBalance_ErrSum<-7200)	sfBalance_ErrSum = -7200;
	iBalance_PWM = (int16_t)((-Car.PID.Balance_Kp * fBias) + (-Car.PID.Balance_Ki * Car.PID.ErrSum ) + (-Car.PID.Balance_Kd * pitch));
	return iBalance_PWM ;
}


/*
*********************************************************************************************************
*                       Car_VelocityPIDCalc                   
*
* Description: 小车速度环PID计算
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
int16_t Car_VelocityPIDCalc(int16_t LeftSpeed, int16_t RightSpeed)
{
	static float Veclocity_Least, Encoder, Encoder_Integral;
	int16_t Velocity;
	
	
	Veclocity_Least = LeftSpeed + RightSpeed - 0;
	
	Encoder *= 0.7;
	Encoder += (Veclocity_Least * 0.3);
	Encoder_Integral += Encoder;
	
	if(Encoder_Integral > 7200) Encoder_Integral = 7200;			//积分限幅
	else if(Encoder_Integral < -7200) Encoder_Integral = -7200;		//
	
	/*  如果小车被放倒,则清除积分  */
	if((Car.MPU.Pitch - Car.BalanceAngle) < 10 || (Car.MPU.Pitch- Car.BalanceAngle) > 40) Encoder_Integral = 0;
	
	Velocity = (int16_t)(Encoder * (Car.PID.Velocity_Kp) + Encoder_Integral * (Car.PID.Velocity_Ki));	//速度环PID计算	
	return Velocity;
}

/*
*********************************************************************************************************
*                      Car_TurnPIDCalc                    
*
* Description: 小车转向环PID计算
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void Car_TurnPIDCalc(void)
{
	
}


/*
*********************************************************************************************************
*                       Car_MotorControl                   
*
* Description: 当PID计算完后调用该函数进行电机控制
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void Car_MotorControl(void)
{
	bsp_motor_SetPwm(Car.Motor.LeftPwm, Car.Motor.RightPwm);
}
/*
*********************************************************************************************************
*                       Car_Control                   
*
* Description: 车子控制函数,解算传感器数据
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/

/*  该控制函数中的控制逻辑并非最佳控制逻辑,需要优化,只是一个模版  */
/*  对于直立小车来说,需要有三个PID计算,直接PID,速度PID,转向PID  */
void Car_Control(void)
{
	int16_t PWM = 0;
	/*  控制函数中翻转LED状态,由此可以测量该函数运行时间  */
	bsp_led_Toggle(1);
		
	/*  处理MPU数据,得到小车当前角度,加速度等  */
//	bsp_mpu_DataProcess();
	
	/*  小车电磁传感器数据处理,得到各自电感的归一化值,以及其他数据  */
	bsp_sensor_DataProcess();
	
	/*  小车电机数据处理  */
	bsp_encoder_SpeedCalc();
	
	/*  对处理完的数据进行PID计算,调整小车姿态  */
	PWM = Car_BalancePIDCalc(Car.MPU.Pitch, Car.MPU.Gryoy);			/*  首先进行直立环PID计算  */
	PWM += Car_VelocityPIDCalc(Car.Motor.LeftSpeed, Car.Motor.RightSpeed);		/*  速度环  */
	Car_TurnPIDCalc();				/*  转向环  */
	if(PWM > 300) PWM = 300;
	if(PWM <- 300) PWM = -300;
	Car.Motor.LeftPwm = PWM;
	Car.Motor.RightPwm = PWM;
	/*  将PID计算结果输出到小车电机  */
	Car_MotorControl();
}
	
	
/********************************************  END OF FILE  *******************************************/
	

