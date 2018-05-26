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
# include "headfile.h"

/*  小车目标速度  */
# define CAR_TARGET_SPEED 40

/*  静止时Z轴角速度漂移  */
# define MPU_GRYOZ_ZERO	-42

/*  速度控制周期,30ms  */
# define SPEED_CONTROL_PERIOD	100	

/*  转向控制周期,5ms  */
# define DIRCTION_CONTROL_PERIOD	5


/*  小车控制结构体,控制小车掺需要用到的数据全部在这个结构体里面,且可以直接使用  */
Car_TypeDef Car;

/*  小车速度环控制PWM输出  */
static float g_SpeedControlOut = 0;						/*  最终的速度环输出  */
static float g_SpeedControlOutNew = 0;				/*  本次速度环的输出  */
static float g_SpeedControlOutOld = 0;				/*  上次速度环的输出  */
static uint16_t g_SpeedControlPeriod = 0;			/*  速度控制周期计数器,用于将速度环的输出平滑处理  */

/*  小车方向控制变量  */
static float g_DirectionControlOut = 0;				/*  最终方向环的输出  */
static float g_DirectionControlOutNew = 0;		/*  本次方向环的输出  */
static float g_DirciotnControlOutOld = 0;			/*  上次方向环的输出  */
static uint16_t g_DirectionControlPeriod = 0;	/*  方向控制周期计数器,用于将方向控制的输出平滑处理  */

/*  直立环控制变量  */
static float g_BalanceControlOut = 0;

/*  冲出跑道计数器,该值大于阈值时,说明冲出跑道了,停车  */
static uint16_t g_LoseLineCounter = 0;

static int16_t g_TargetSpeedControl = 0;

static float g_GryozKd = 0;			/*  角速度控制转向系数  */


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
	
	
		/*  初始化方向模糊PID参数  */
	Car.DirFuzzy.DeltaKdMax = 10;
	Car.DirFuzzy.DeltaKiMax = 0;
	Car.DirFuzzy.DeltaKpMax = 0.8;
	Car.DirFuzzy.DErrMax = 10;
	Car.DirFuzzy.ErrMax = 90;
	Car.DirFuzzy.KP = 3;
	Car.DirFuzzy.KD = 180;
	Car.DirFuzzy.KPMax = 5.5;//Car.PID.DirectionKp;
	Car.DirFuzzy.KIMax = 0;//Car.PID.DirectionKi;
	Car.DirFuzzy.KDMax = 400;//Car.PID.DirectionKd;
	fuzzy_PIDInit(&Car.DirFuzzy);
	
	
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
	Car.Motor.LeftPwm = 0;
	Car.Motor.RightPwm = 0;
	Car.Motor.LeftEncoder = 0;
	Car.Motor.RightEncoder = 0;
	Car.Motor.LeftSpeed = 0;
	Car.Motor.RightSpeed = 0;
	
	Car.TargetSpeed = 0;

	Car.MaxPWM = 800;
	Car.BalanceAngle = 7.2;
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
*                                          
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
void Car_ControlStop(void)
{
}


/*
*********************************************************************************************************
*                                          
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
void Car_Reset(void)
{
}

/*
*********************************************************************************************************
*                                          
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
void Car_ControlStart(void)
{
}

/*
*********************************************************************************************************
*                                          
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
void Car_ParaStore(void)
{
}


/*
*********************************************************************************************************
*                       Car_BalanceControl                   
*
* Description: 小车直立环控制
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void Car_BalanceControl(void)
{
	float Error = 0, ErrorDiff = 0;   /*  每个时刻的偏差,以及偏差微分 */
	float Kp = 0, Kd = 0;
	
	/*  计算偏差  */
	Error = Car.BalanceAngle - Car.MPU.Pitch;

	ErrorDiff = Car.MPU.Gyroy;
	
	Kp = Car.PID.Balance_Kp;
	Kd = Car.PID.Balance_Kd;
	
	/*  直立环的控制采用PD控制,输入为直立偏差,以及直立角速度  */
	g_BalanceControlOut = Kp * Error - Kd * ErrorDiff;
}

/*
*********************************************************************************************************
*                          Car_SpeedControl                
*
* Description: 小车速度环控制,采用积分分离PI控制
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void Car_SpeedControl(void)
{
	static float SpeedFilter, SpeedIntegal;
	int32_t LeftEnconder = 0, RightEnconder = 0;
	float SpeedError = 0;
	float Kp = 0, Ki = 0, Kd = 0;
	
	/*  由于两个编码器旋转了180度,所以有一个极性差  */
	LeftEnconder = (READ_DIR(LEFTENCONDER_DIR_PIN) == 0) ? (-Car.Motor.LeftEncoder) : Car.Motor.LeftEncoder;
	RightEnconder = (READ_DIR(RIGHTENCONDER_DIR_PIN) == 1) ? (-Car.Motor.RightEncoder) : (Car.Motor.RightEncoder);
	Car.Motor.LeftEncoder = 0;
	Car.Motor.RightEncoder = 0;
	
	/*  将速度进行转换,计算成转/秒  */
	Car.CarSpeed = (LeftEnconder + RightEnconder) / 2 * CAR_SPEED_CONSTANT;
		
		/*  速度偏差  */
	SpeedError = Car.TargetSpeed -Car.CarSpeed;
	
	/*  低通滤波,让速度平滑过渡  */
	SpeedFilter *= 0.7;
	SpeedFilter += (SpeedError * 0.3);
	
	if(SpeedError < 10 && SpeedError >= -10)
		SpeedIntegal += SpeedFilter;
	
	/*  积分限幅  */
	if(SpeedIntegal > 3000) SpeedIntegal = 3000;			//积分限幅
	else if(SpeedIntegal < -3000) SpeedIntegal = -3000;		//
	
	Kp = Car.PID.Velocity_Kp;
	Ki = Car.PID.Velocity_Ki;
	Kd = Car.PID.Velocity_Kd;
	
	/*  速度环PI控制 */
	g_SpeedControlOutOld = g_SpeedControlOutNew;
	g_SpeedControlOutNew = SpeedFilter * Kp +	SpeedIntegal * Ki;

}
/*
*********************************************************************************************************
*                          Car_SpeedControlOutput                
*
* Description: 将速度环的输出分为速度控制周期的n等份输出到电机,可以让速度变化更平滑
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void Car_SpeedControlOutput(void)
{
	float SpeedControlValue;
	
	SpeedControlValue = g_SpeedControlOutNew - g_SpeedControlOutOld;
	g_SpeedControlOut = SpeedControlValue * (g_SpeedControlPeriod + 1) / SPEED_CONTROL_PERIOD + g_SpeedControlOutOld;
}

/*
*********************************************************************************************************
*                        Car_DirctionControl                  
*
* Description: 小车方向环控制
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void Car_DirctionControl(void)
{
	static float LastError = 0;
	float Error = 0, ErrorDiff = 0, Gyro_Z = 0;
	float Kp = 0, Kd = 0;
	
	Error = Car.HorizontalAE;
	ErrorDiff = Error - LastError;
	
	Gyro_Z = Car.MPU.Gyroy - MPU_GRYOZ_ZERO;
	
	/*  由于模糊PID算出来的值有负的,所以需要极性判断  */
	fuzzy_PIDClac(&Car.DirFuzzy, Error, ErrorDiff);
	
	Kp = (Car.DirFuzzy.KP < 0) ? (- Car.DirFuzzy.KP) : Car.DirFuzzy.KP;
	Kd = (Car.DirFuzzy.KD < 0) ? (- Car.DirFuzzy.KD) : Car.DirFuzzy.KD;

//	Kp = Car.PID.Turn_Kp;
//	Kd = Car.PID.Turn_Kd;
	
	/*  保存上次的PWM  */
	g_DirciotnControlOutOld = g_DirectionControlOutNew; 
	
	/*  计算PWM.利用陀螺仪的角速度来控制转向  */
	g_DirectionControlOutNew = -Error *  Kp + ErrorDiff * -Kd + g_GryozKd * Gyro_Z;
	
	if(Car.HorizontalAE < 4 && Car.HorizontalAE>-4) g_DirectionControlOutNew = 0;
	
	/*  保存上个时刻的误差  */
	LastError = Error;
}
/*
*********************************************************************************************************
*                         Car_DirctionControlOutput                 
*
* Description: 将转向环的输出分为转向控制周期的n等份进行输出,让速度变化平滑
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void Car_DirctionControlOutput(void)
{
	float DirectionOutput = 0;
	
	DirectionOutput = g_DirectionControlOutNew - g_DirciotnControlOutOld;
	g_DirectionControlOut = DirectionOutput * (g_DirectionControlPeriod + 1) / DIRCTION_CONTROL_PERIOD + g_DirciotnControlOutOld;
}

/*
*********************************************************************************************************
*                            Car_MotorOutput              
*
* Description: 将速度环,转向环,直立环的输出进行叠加,输出到电机
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/

void Car_MotorOutput(void)
{
	int16_t LeftPwm = 0, RightPwm = 0;
	
	/*  电机控制PWM为平衡环,速度环,转向环叠加  */
	LeftPwm = (int16_t)(g_BalanceControlOut + g_SpeedControlOut - g_DirectionControlOut);
	RightPwm = (int16_t)(g_BalanceControlOut + g_SpeedControlOut + g_DirectionControlOut);
	
	/*  增加死区  */
	if(LeftPwm > 0) LeftPwm += MOTOR_DEAD_VAL;
	else if(LeftPwm < 0) LeftPwm -= MOTOR_DEAD_VAL;
	
	if(RightPwm > 0) RightPwm += (MOTOR_DEAD_VAL);
	else if(RightPwm < 0) RightPwm -= (MOTOR_DEAD_VAL);
	
	/*  对PWM进行限幅  */
	if(LeftPwm > Car.MaxPWM) LeftPwm = Car.MaxPWM;
	else if(LeftPwm < -Car.MaxPWM) LeftPwm = -Car.MaxPWM;
	
	if(RightPwm > Car.MaxPWM) RightPwm = Car.MaxPWM;
	else if(RightPwm < -Car.MaxPWM) RightPwm = -Car.MaxPWM;
		
	/*  保存计算结果  */
	Car.Motor.LeftPwm = LeftPwm;
	Car.Motor.RightPwm = RightPwm;
	
	/*  检测是否冲出跑道  */
//	if(Car.Sensor[SENSOR_H_L].Average < 15 && Car.Sensor[SENSOR_H_R].Average < 15)
//		g_LoseLineCounter ++;
	
	/*  当车子前倾角或后倾角过大时停车,  */
	if((Car.MPU.Pitch - Car.BalanceAngle) > 30 || (Car.MPU.Pitch - Car.BalanceAngle) < -9)
	{
		bsp_motor_SetPwm(0,0);
		g_TargetSpeedControl = 0;
		Car.TargetSpeed = 0;
	} 
//	else if(g_LoseLineCounter > 2)		/*  冲出跑道时停车  */
//	{
//		g_TargetSpeedControl = 0;
//		Car.TargetSpeed = 0;
//	}
	else
	{
		/*  输出到电机  */
		bsp_motor_SetPwm(Car.Motor.LeftPwm, Car.Motor.RightPwm);
	}
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
	static uint16_t Car_ControlCounter = 0;
	static uint16_t SpeedControlCounter = 0;
	static uint16_t DirectionControlCounter = 0;
	
	
	Car_ControlCounter++;
	
	/*  将速度环的输出均分  */
	g_SpeedControlPeriod++;
	Car_SpeedControlOutput();
	
	/*  将转向环的输出均分  */
	g_DirectionControlPeriod++;
	Car_DirctionControlOutput();
	
	/*  小车控制状态机  */
	switch(Car_ControlCounter)
	{
		case 1:		/*  每5ms读一次编码器的值  */
		{
			bsp_encoder_ReadCounter();
		}break;
		case 2: 
		{
			SpeedControlCounter++;
			/*  因为每5ms进入一次,所以进行除5  */
			if(SpeedControlCounter >= SPEED_CONTROL_PERIOD/5)		/*  速度控制  */
			{
				SpeedControlCounter = 0;
				g_SpeedControlPeriod = 0;
				g_TargetSpeedControl ++;
				if(Car.TargetSpeed < CAR_TARGET_SPEED)
				{
					Car.TargetSpeed = CAR_TARGET_SPEED*(g_TargetSpeedControl+1)/20;
				}
				Car_SpeedControl();
			}
		}break;
		case 3: bsp_sensor_DataProcess(); break;		/*  5ms进行一次传感器的数据处理  */
		case 4: 
		{
			DirectionControlCounter++;
			if(DirectionControlCounter >= DIRCTION_CONTROL_PERIOD/5)	/*  方向控制  */
			{
				DirectionControlCounter = 0;
				g_DirectionControlPeriod = 0;
				Car_DirctionControl();
			}
		}break;
		case 5: 		/*  5ms进行一次电机输出,以及直立控制  */
		{
			Car_BalanceControl();
			Car_MotorOutput();
			Car_ControlCounter = 0;
		}break;
		default: 
		{
			DirectionControlCounter = 0;
			g_SpeedControlPeriod = 0;
			SpeedControlCounter = 0;
			g_DirectionControlPeriod = 0;
			Car_ControlCounter = 0;
		}break;
	}
}
	
	
/********************************************  END OF FILE  *******************************************/
	

