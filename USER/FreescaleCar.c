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

/*  С�����ƽṹ��,����С������Ҫ�õ�������ȫ��������ṹ������,�ҿ���ֱ��ʹ��  */
Car_TypeDef Car;


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
	Car.Motor.PWM_Frequency = 10;	/*  ���PWMƵ��Ϊ10KHz  */
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
*                        Car_BalancePIDCalc                  
*
* Description: С��ֱ����PID����
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
	float fBias;   //ÿ��ʱ�̵�ƫ��
	int16_t iBalance_PWM; //������PWM
	static float sfBalance_ErrSum;//	
	
	fBias = angle - Car.BalanceAngle;//ƫ����ڵ�ǰ�Ƕȼ�ȥ��ʼ��ʱ��ƫ����ټ�ȥ�޵������ʱ��ֱ���ĽǶ�
	
	Car.PID.ErrSum += fBias; //�Ժʹ������
	
	if(sfBalance_ErrSum > 7200) sfBalance_ErrSum = 7200;  //�����޷�
	else if(sfBalance_ErrSum<-7200)	sfBalance_ErrSum = -7200;
	iBalance_PWM = (int16_t)((-Car.PID.Balance_Kp * fBias) + (-Car.PID.Balance_Ki * Car.PID.ErrSum ) + (-Car.PID.Balance_Kd * pitch));
	return iBalance_PWM ;
}


/*
*********************************************************************************************************
*                       Car_VelocityPIDCalc                   
*
* Description: С���ٶȻ�PID����
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
	
	if(Encoder_Integral > 7200) Encoder_Integral = 7200;			//�����޷�
	else if(Encoder_Integral < -7200) Encoder_Integral = -7200;		//
	
	/*  ���С�����ŵ�,���������  */
	if((Car.MPU.Pitch - Car.BalanceAngle) < 10 || (Car.MPU.Pitch- Car.BalanceAngle) > 40) Encoder_Integral = 0;
	
	Velocity = (int16_t)(Encoder * (Car.PID.Velocity_Kp) + Encoder_Integral * (Car.PID.Velocity_Ki));	//�ٶȻ�PID����	
	return Velocity;
}

/*
*********************************************************************************************************
*                      Car_TurnPIDCalc                    
*
* Description: С��ת��PID����
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
* Description: ��PID���������øú������е������
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
	int16_t PWM = 0;
	/*  ���ƺ����з�תLED״̬,�ɴ˿��Բ����ú�������ʱ��  */
	bsp_led_Toggle(1);
		
	/*  ����MPU����,�õ�С����ǰ�Ƕ�,���ٶȵ�  */
//	bsp_mpu_DataProcess();
	
	/*  С����Ŵ��������ݴ���,�õ����Ե�еĹ�һ��ֵ,�Լ���������  */
	bsp_sensor_DataProcess();
	
	/*  С��������ݴ���  */
	bsp_encoder_SpeedCalc();
	
	/*  �Դ���������ݽ���PID����,����С����̬  */
	PWM = Car_BalancePIDCalc(Car.MPU.Pitch, Car.MPU.Gryoy);			/*  ���Ƚ���ֱ����PID����  */
	PWM += Car_VelocityPIDCalc(Car.Motor.LeftSpeed, Car.Motor.RightSpeed);		/*  �ٶȻ�  */
	Car_TurnPIDCalc();				/*  ת��  */
	if(PWM > 300) PWM = 300;
	if(PWM <- 300) PWM = -300;
	Car.Motor.LeftPwm = PWM;
	Car.Motor.RightPwm = PWM;
	/*  ��PID�����������С�����  */
	Car_MotorControl();
}
	
	
/********************************************  END OF FILE  *******************************************/
	

