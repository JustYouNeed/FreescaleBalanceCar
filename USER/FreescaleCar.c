/**
  *******************************************************************************************************
  * File Name: FreescaleCar.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-3-3
  * Brief: ���ļ����ڳ������ݼ�¼����������ȣ�ͬʱ���ӿ��ƺ���Ҳ�ڱ��ļ�
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-4
	*			Mod: �����ļ�
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

/*  С��Ŀ���ٶ�  */
# define CAR_TARGET_SPEED 40

/*  ��ֹʱZ����ٶ�Ư��  */
# define MPU_GRYOZ_ZERO	-42

/*  �ٶȿ�������,30ms  */
# define SPEED_CONTROL_PERIOD	100	

/*  ת���������,5ms  */
# define DIRCTION_CONTROL_PERIOD	5


/*  С�����ƽṹ��,����С������Ҫ�õ�������ȫ��������ṹ������,�ҿ���ֱ��ʹ��  */
Car_TypeDef Car;

/*  С���ٶȻ�����PWM���  */
static float g_SpeedControlOut = 0;						/*  ���յ��ٶȻ����  */
static float g_SpeedControlOutNew = 0;				/*  �����ٶȻ������  */
static float g_SpeedControlOutOld = 0;				/*  �ϴ��ٶȻ������  */
static uint16_t g_SpeedControlPeriod = 0;			/*  �ٶȿ������ڼ�����,���ڽ��ٶȻ������ƽ������  */

/*  С��������Ʊ���  */
static float g_DirectionControlOut = 0;				/*  ���շ��򻷵����  */
static float g_DirectionControlOutNew = 0;		/*  ���η��򻷵����  */
static float g_DirciotnControlOutOld = 0;			/*  �ϴη��򻷵����  */
static uint16_t g_DirectionControlPeriod = 0;	/*  ����������ڼ�����,���ڽ�������Ƶ����ƽ������  */

/*  ֱ�������Ʊ���  */
static float g_BalanceControlOut = 0;

/*  ����ܵ�������,��ֵ������ֵʱ,˵������ܵ���,ͣ��  */
static uint16_t g_LoseLineCounter = 0;

static int16_t g_TargetSpeedControl = 0;

static float g_GryozKd = 0;			/*  ���ٶȿ���ת��ϵ��  */


/*
*********************************************************************************************************
*                         Car_ParaInit                 
*
* Description: ���Ӳ�����ʼ��,��оƬFlash�ж�ȡ���洢�ĵ������
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
	
	/*  ��ʼ�����ӵĸ����  */
	Car.HorizontalAE = 0;
	Car.VecticalAE = 0;
		
	/*  ��ʼ�����ӵ�PID����,��Flash�ж�ȡ�������PID����  */
	Car.PID.Balance_Kp = flash_read(PID_PARA_FLASH_ADDR, 0, float);	/*  ֱ��PID����  */
	Car.PID.Balance_Ki = flash_read(PID_PARA_FLASH_ADDR, 4, float);
	Car.PID.Balance_Kd = flash_read(PID_PARA_FLASH_ADDR, 8, float);
	
	Car.PID.Velocity_Kp = flash_read(PID_PARA_FLASH_ADDR, 12, float);		/*  �ٶȻ�PID����  */
	Car.PID.Velocity_Ki = flash_read(PID_PARA_FLASH_ADDR, 16, float);
	Car.PID.Velocity_Kd = flash_read(PID_PARA_FLASH_ADDR, 20, float);
	
	Car.PID.Turn_Kp = flash_read(PID_PARA_FLASH_ADDR, 24, float);		/*  ת��PID����  */
	Car.PID.Turn_Ki = flash_read(PID_PARA_FLASH_ADDR, 28, float);
	Car.PID.Turn_Kd = flash_read(PID_PARA_FLASH_ADDR, 32, float);
	
	
		/*  ��ʼ������ģ��PID����  */
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
	
	
	/*  ��ʼ�����ӵĴ���������,��Flash�ж�ȡ�궨ֵ  */
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
	
	/*  ������Ʋ�����ʼ��  */
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
* Description: С������ָʾ,LED����˸
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
* Description: С��ֱ��������
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
	float Error = 0, ErrorDiff = 0;   /*  ÿ��ʱ�̵�ƫ��,�Լ�ƫ��΢�� */
	float Kp = 0, Kd = 0;
	
	/*  ����ƫ��  */
	Error = Car.BalanceAngle - Car.MPU.Pitch;

	ErrorDiff = Car.MPU.Gyroy;
	
	Kp = Car.PID.Balance_Kp;
	Kd = Car.PID.Balance_Kd;
	
	/*  ֱ�����Ŀ��Ʋ���PD����,����Ϊֱ��ƫ��,�Լ�ֱ�����ٶ�  */
	g_BalanceControlOut = Kp * Error - Kd * ErrorDiff;
}

/*
*********************************************************************************************************
*                          Car_SpeedControl                
*
* Description: С���ٶȻ�����,���û��ַ���PI����
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
	
	/*  ����������������ת��180��,������һ�����Բ�  */
	LeftEnconder = (READ_DIR(LEFTENCONDER_DIR_PIN) == 0) ? (-Car.Motor.LeftEncoder) : Car.Motor.LeftEncoder;
	RightEnconder = (READ_DIR(RIGHTENCONDER_DIR_PIN) == 1) ? (-Car.Motor.RightEncoder) : (Car.Motor.RightEncoder);
	Car.Motor.LeftEncoder = 0;
	Car.Motor.RightEncoder = 0;
	
	/*  ���ٶȽ���ת��,�����ת/��  */
	Car.CarSpeed = (LeftEnconder + RightEnconder) / 2 * CAR_SPEED_CONSTANT;
		
		/*  �ٶ�ƫ��  */
	SpeedError = Car.TargetSpeed -Car.CarSpeed;
	
	/*  ��ͨ�˲�,���ٶ�ƽ������  */
	SpeedFilter *= 0.7;
	SpeedFilter += (SpeedError * 0.3);
	
	if(SpeedError < 10 && SpeedError >= -10)
		SpeedIntegal += SpeedFilter;
	
	/*  �����޷�  */
	if(SpeedIntegal > 3000) SpeedIntegal = 3000;			//�����޷�
	else if(SpeedIntegal < -3000) SpeedIntegal = -3000;		//
	
	Kp = Car.PID.Velocity_Kp;
	Ki = Car.PID.Velocity_Ki;
	Kd = Car.PID.Velocity_Kd;
	
	/*  �ٶȻ�PI���� */
	g_SpeedControlOutOld = g_SpeedControlOutNew;
	g_SpeedControlOutNew = SpeedFilter * Kp +	SpeedIntegal * Ki;

}
/*
*********************************************************************************************************
*                          Car_SpeedControlOutput                
*
* Description: ���ٶȻ��������Ϊ�ٶȿ������ڵ�n�ȷ���������,�������ٶȱ仯��ƽ��
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
* Description: С�����򻷿���
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
	
	/*  ����ģ��PID�������ֵ�и���,������Ҫ�����ж�  */
	fuzzy_PIDClac(&Car.DirFuzzy, Error, ErrorDiff);
	
	Kp = (Car.DirFuzzy.KP < 0) ? (- Car.DirFuzzy.KP) : Car.DirFuzzy.KP;
	Kd = (Car.DirFuzzy.KD < 0) ? (- Car.DirFuzzy.KD) : Car.DirFuzzy.KD;

//	Kp = Car.PID.Turn_Kp;
//	Kd = Car.PID.Turn_Kd;
	
	/*  �����ϴε�PWM  */
	g_DirciotnControlOutOld = g_DirectionControlOutNew; 
	
	/*  ����PWM.���������ǵĽ��ٶ�������ת��  */
	g_DirectionControlOutNew = -Error *  Kp + ErrorDiff * -Kd + g_GryozKd * Gyro_Z;
	
	if(Car.HorizontalAE < 4 && Car.HorizontalAE>-4) g_DirectionControlOutNew = 0;
	
	/*  �����ϸ�ʱ�̵����  */
	LastError = Error;
}
/*
*********************************************************************************************************
*                         Car_DirctionControlOutput                 
*
* Description: ��ת�򻷵������Ϊת��������ڵ�n�ȷݽ������,���ٶȱ仯ƽ��
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
* Description: ���ٶȻ�,ת��,ֱ������������е���,��������
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
	
	/*  �������PWMΪƽ�⻷,�ٶȻ�,ת�򻷵���  */
	LeftPwm = (int16_t)(g_BalanceControlOut + g_SpeedControlOut - g_DirectionControlOut);
	RightPwm = (int16_t)(g_BalanceControlOut + g_SpeedControlOut + g_DirectionControlOut);
	
	/*  ��������  */
	if(LeftPwm > 0) LeftPwm += MOTOR_DEAD_VAL;
	else if(LeftPwm < 0) LeftPwm -= MOTOR_DEAD_VAL;
	
	if(RightPwm > 0) RightPwm += (MOTOR_DEAD_VAL);
	else if(RightPwm < 0) RightPwm -= (MOTOR_DEAD_VAL);
	
	/*  ��PWM�����޷�  */
	if(LeftPwm > Car.MaxPWM) LeftPwm = Car.MaxPWM;
	else if(LeftPwm < -Car.MaxPWM) LeftPwm = -Car.MaxPWM;
	
	if(RightPwm > Car.MaxPWM) RightPwm = Car.MaxPWM;
	else if(RightPwm < -Car.MaxPWM) RightPwm = -Car.MaxPWM;
		
	/*  ���������  */
	Car.Motor.LeftPwm = LeftPwm;
	Car.Motor.RightPwm = RightPwm;
	
	/*  ����Ƿ����ܵ�  */
//	if(Car.Sensor[SENSOR_H_L].Average < 15 && Car.Sensor[SENSOR_H_R].Average < 15)
//		g_LoseLineCounter ++;
	
	/*  ������ǰ��ǻ����ǹ���ʱͣ��,  */
	if((Car.MPU.Pitch - Car.BalanceAngle) > 30 || (Car.MPU.Pitch - Car.BalanceAngle) < -9)
	{
		bsp_motor_SetPwm(0,0);
		g_TargetSpeedControl = 0;
		Car.TargetSpeed = 0;
	} 
//	else if(g_LoseLineCounter > 2)		/*  ����ܵ�ʱͣ��  */
//	{
//		g_TargetSpeedControl = 0;
//		Car.TargetSpeed = 0;
//	}
	else
	{
		/*  ��������  */
		bsp_motor_SetPwm(Car.Motor.LeftPwm, Car.Motor.RightPwm);
	}
}

/*
*********************************************************************************************************
*                       Car_Control                   
*
* Description: ���ӿ��ƺ���,���㴫��������
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/

/*  �ÿ��ƺ����еĿ����߼�������ѿ����߼�,��Ҫ�Ż�,ֻ��һ��ģ��  */
/*  ����ֱ��С����˵,��Ҫ������PID����,ֱ��PID,�ٶ�PID,ת��PID  */
void Car_Control(void)
{
	static uint16_t Car_ControlCounter = 0;
	static uint16_t SpeedControlCounter = 0;
	static uint16_t DirectionControlCounter = 0;
	
	
	Car_ControlCounter++;
	
	/*  ���ٶȻ����������  */
	g_SpeedControlPeriod++;
	Car_SpeedControlOutput();
	
	/*  ��ת�򻷵��������  */
	g_DirectionControlPeriod++;
	Car_DirctionControlOutput();
	
	/*  С������״̬��  */
	switch(Car_ControlCounter)
	{
		case 1:		/*  ÿ5ms��һ�α�������ֵ  */
		{
			bsp_encoder_ReadCounter();
		}break;
		case 2: 
		{
			SpeedControlCounter++;
			/*  ��Ϊÿ5ms����һ��,���Խ��г�5  */
			if(SpeedControlCounter >= SPEED_CONTROL_PERIOD/5)		/*  �ٶȿ���  */
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
		case 3: bsp_sensor_DataProcess(); break;		/*  5ms����һ�δ����������ݴ���  */
		case 4: 
		{
			DirectionControlCounter++;
			if(DirectionControlCounter >= DIRCTION_CONTROL_PERIOD/5)	/*  �������  */
			{
				DirectionControlCounter = 0;
				g_DirectionControlPeriod = 0;
				Car_DirctionControl();
			}
		}break;
		case 5: 		/*  5ms����һ�ε�����,�Լ�ֱ������  */
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
	

