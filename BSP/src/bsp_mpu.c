# include "bsp_mpu.h"
# include "FreescaleCar.h"
# include "app_filter.h"
# include "math.h"
# include "bsp_i2c.h"


# define PI	3.14159265f

/*  MPU IIC控制结构体  */
IIC_TypeDef mpu_iic;
static Kalman_TypeDef PitchKalman; 
static Kalman_TypeDef RollKalman; 
static Kalman_TypeDef YawKalman; 

/*
*********************************************************************************************************
*                      MPU_SDA_HIGH                    
*
* Description: 设置SDA引脚电平为高
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void MPU_SDA_HIGH(void)
{
	gpio_set(MPU_SDA_PIN, 1);
}


/*
*********************************************************************************************************
*                      MPU_SDA_LOW                    
*
* Description: 设置SDA引脚电平为低
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void MPU_SDA_LOW(void)
{
	gpio_set(MPU_SDA_PIN, 0);
}
/*
*********************************************************************************************************
*                      MPU_SDA_SET_OUT                    
*
* Description: 设置SDA引脚为输出模式
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void MPU_SDA_SET_OUT(void)
{
	gpio_init(MPU_SDA_PIN, GPO, 1);
}

/*
*********************************************************************************************************
*                      MPU_SDA_SET_IN                    
*
* Description: 设置SDA引脚为输入模式
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void MPU_SDA_SET_IN(void)
{
	gpio_init(MPU_SDA_PIN, GPI, 1);
}
/*
*********************************************************************************************************
*                      MPU_SCL_HIGH                    
*
* Description: 设置SCL引脚电平为高
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void MPU_SCL_HIGH(void)
{
	gpio_set(MPU_SCL_PIN, 1);
}
/*
*********************************************************************************************************
*                      MPU_SCL_LOW                    
*
* Description: 设置SCL引脚电平为低
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void MPU_SCL_LOW(void)
{
	gpio_set(MPU_SCL_PIN, 0);
}
/*
*********************************************************************************************************
*                      MPU_READ_SDA                    
*
* Description: 读取SDA引脚电平值
*             
* Arguments  : None.
*
* Reutrn     : SDA引脚电平值
*
* Note(s)    : None.
*********************************************************************************************************
*/
static uint8_t MPU_READ_SDA(void)
{
	return gpio_get(MPU_SDA_PIN);
}

/*
*********************************************************************************************************
*                              bsp_mpu_IICConfig            
*
* Description: 初始化MPU的IIC
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None
*********************************************************************************************************
*/
void bsp_mpu_IICConfig(void)
{
	gpio_init(MPU_SDA_PIN, GPI, 1);
	port_pull(MPU_SDA_PIN);
	gpio_init(MPU_SCL_PIN, GPO, 1);
	port_pull(MPU_SCL_PIN);
	
	mpu_iic.set_scl_high = MPU_SCL_HIGH;
	mpu_iic.set_scl_low = MPU_SCL_LOW;
	mpu_iic.set_sda_high = MPU_SDA_HIGH;
	mpu_iic.set_sda_low = MPU_SDA_LOW;
	mpu_iic.set_sda_in = MPU_SDA_SET_IN;
	mpu_iic.set_sda_out = MPU_SDA_SET_OUT;
	mpu_iic.read_sda = MPU_READ_SDA;
}
/*
*********************************************************************************************************
*                         bsp_mpu_Config                 
*
* Description: MPU初始化函数
*             
* Arguments  : 
*
* Reutrn     : 
*
* Note(s)    : 
*********************************************************************************************************
*/
uint8_t bsp_mpu_Config(void)
{
	uint8_t res = 0;
	
	bsp_mpu_IICConfig();
	
	bsp_mpu_WriteByte(MPU_PWR_MGMT1_REG, 0x80);
	bsp_tim_DelayMs(200);
	bsp_mpu_WriteByte(MPU_PWR_MGMT1_REG, 0x00);
	bsp_mpu_SetGyroFsr(3);
	bsp_mpu_SetAccelFsr(0);
	bsp_mpu_SetRate(100);
	
	bsp_mpu_WriteByte(MPU_USER_CTRL_REG, 0x00);
	bsp_mpu_WriteByte(MPU_FIFO_EN_REG, 0x00);
	bsp_mpu_WriteByte(MPU_INTBP_CFG_REG, 0x80);
	bsp_mpu_WriteByte(MPU_INT_EN_REG, 0x00);
	
	res = bsp_mpu_ReadByte(MPU_DEVICE_ID_REG);
	
	/*  读取MPU地址以判断是否初始化成功  */
	if(MPU_ADDR == res)
	{
		bsp_mpu_WriteByte(MPU_PWR_MGMT1_REG, 0x01);
		bsp_mpu_WriteByte(MPU_PWR_MGMT2_REG, 0x00);
		bsp_mpu_SetRate(100);
	}else
		return 1;
	
	filter_KanlmanInit(&RollKalman);
	filter_KanlmanInit(&PitchKalman);
	filter_KanlmanInit(&YawKalman);
	
//	while(mpu_dmp_init());	/*  等待DMP库初始化完成  */
	
	return 0;

}

/*
*********************************************************************************************************
*                     bsp_mpu_WriteByte                    
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
uint8_t bsp_mpu_WriteByte(uint8_t reg, uint8_t byte)
{
	bsp_i2c_Start(mpu_iic);
	bsp_i2c_SendByte(mpu_iic, (MPU_ADDR << 1) | 0);
	
	if(bsp_i2c_WaitAck(mpu_iic))
	{
		bsp_i2c_Stop(mpu_iic);
		return 1;
	}
	
	bsp_i2c_SendByte(mpu_iic, reg);
	bsp_i2c_WaitAck(mpu_iic);
	bsp_i2c_SendByte(mpu_iic, byte);
	if(bsp_i2c_WaitAck(mpu_iic))
	{
		bsp_i2c_Stop(mpu_iic);
		
		return 1;
	}
	bsp_i2c_Stop(mpu_iic);
	return 0;
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
uint8_t bsp_mpu_ReadByte(uint8_t reg)
{
	uint8_t result = 0x00;
	
	bsp_i2c_Start(mpu_iic);
	bsp_i2c_SendByte(mpu_iic, (MPU_ADDR << 1) | 0);
	bsp_i2c_WaitAck(mpu_iic);
	
	bsp_i2c_SendByte(mpu_iic, reg);
	bsp_i2c_WaitAck(mpu_iic);
	
	bsp_i2c_Start(mpu_iic);
	bsp_i2c_SendByte(mpu_iic, (MPU_ADDR << 1) | 1);
	bsp_i2c_WaitAck(mpu_iic);
	
	result = bsp_i2c_ReadByte(mpu_iic, 0);
	bsp_i2c_Stop(mpu_iic);
	return result;
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
uint8_t bsp_mpu_WriteBuff(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buff)
{
	uint8_t cnt = 0;
	bsp_i2c_Start(mpu_iic);
	bsp_i2c_SendByte(mpu_iic, (MPU_ADDR << 1) | 0);
	if(bsp_i2c_WaitAck(mpu_iic))
	{
		bsp_i2c_Stop(mpu_iic);
		return 1;
	}
	bsp_i2c_SendByte(mpu_iic, reg);
	bsp_i2c_WaitAck(mpu_iic);
	
	for(cnt = 0; cnt < len; cnt ++)
	{
		bsp_i2c_SendByte(mpu_iic, buff[cnt]);
		if(bsp_i2c_WaitAck(mpu_iic))
		{
			bsp_i2c_Stop(mpu_iic);
			return 1;
		}
	}
	
	return 0;
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
uint8_t bsp_mpu_ReadBuff(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buff)
{
	bsp_i2c_Start(mpu_iic);
	bsp_i2c_SendByte(mpu_iic, (MPU_ADDR << 1) | 0);
	if(bsp_i2c_WaitAck(mpu_iic))
	{
		bsp_i2c_Stop(mpu_iic);
		return 1;
	}
	bsp_i2c_SendByte(mpu_iic, reg);
	bsp_i2c_WaitAck(mpu_iic);
	bsp_i2c_Start(mpu_iic);
	bsp_i2c_SendByte(mpu_iic, (MPU_ADDR << 1) | 1);
	bsp_i2c_WaitAck(mpu_iic);
	
	while(len)
	{
		if(len == 1) *buff = bsp_i2c_ReadByte(mpu_iic, 0);
		else *buff = bsp_i2c_ReadByte(mpu_iic, 1);
		
		len --;
		buff ++;
	}
	bsp_i2c_Stop(mpu_iic);
	return 0;
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
uint8_t bsp_mpu_SetGyroFsr(uint8_t fsr)
{
	return bsp_mpu_WriteByte(MPU_GYRO_CFG_REG, fsr<<3);
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
uint8_t bsp_mpu_SetAccelFsr(uint8_t fsr)
{
	return bsp_mpu_WriteByte(MPU_ACCEL_CFG_REG, fsr<<3);
}


/*
*********************************************************************************************************
*                   bsp_mpu_SetLPF                       
*
* Description: 设置MPU的低通滤波器
*             
* Arguments  : 1> lpf: 低通滤波器值
*
* Reutrn     : 1> 读取回来的寄存器值
*
* Note(s)    : None
*********************************************************************************************************
*/
uint8_t bsp_mpu_SetLPF(uint16_t lpf)
{
	uint8_t data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6;
	
	return bsp_mpu_WriteByte(MPU_CFG_REG, data);
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
uint8_t bsp_mpu_SetRate(uint16_t rate)
{
	uint8_t data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	
	data = bsp_mpu_WriteByte(MPU_SAMPLE_RATE_REG, data);
	return bsp_mpu_SetLPF(rate / 2);
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
uint8_t bsp_mpu_SetFIFO(uint8_t sens)
{
	return 0;
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
short bsp_mpu_ReadTemperature(void)
{
	uint8_t buff[2];
	short raw = 0;
	float temp = 0.0f;
	
	bsp_mpu_ReadBuff(MPU_ADDR, MPU_TEMP_OUTH_REG, 2, buff);
	raw = ((uint16_t)buff[0]<<8)|buff[1];
	temp = 36.53+((double)raw)/340; 
	return temp;
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
uint8_t bsp_mpu_ReadGyro(short *gx, short *gy, short* gz)
{
	uint8_t buf[6],res;  
	res=bsp_mpu_ReadBuff(MPU_ADDR,MPU_GYRO_XOUTH_REG, 6, buf);
	if(res==0)
	{
		*gx=((uint16_t)buf[0]<<8)|buf[1];  
		*gy=((uint16_t)buf[2]<<8)|buf[3];  
		*gz=((uint16_t)buf[4]<<8)|buf[5];
	} 	
  return res;;
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
uint8_t bsp_mpu_ReadAcc(short* ax, short*ay, short*az)
{
	uint8_t buf[6],res;  
	res=bsp_mpu_ReadBuff(MPU_ADDR, MPU_ACCEL_XOUTH_REG, 6, buf);
	if(res==0)
	{
		*ax=((uint16_t)buf[0]<<8)|buf[1];  
		*ay=((uint16_t)buf[2]<<8)|buf[3];  
		*az=((uint16_t)buf[4]<<8)|buf[5];
	} 	
	return res;;
}


  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
float K1 =0.02; 
float angle, angle_dot; 	
float Q_angle=0.001;// 过程噪声的协方差
float Q_gyro=0.003;//0.003 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
float R_angle=0.5;// 测量噪声的协方差 既测量偏差
float dt=0.005;//                 
char  C_0 = 1;
float Q_bias, Angle_err;
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;
float Pdot[4] ={0,0,0,0};
float PP[2][2] = { { 1, 0 },{ 0, 1 } };

/**************************************************************************
函数功能：简易卡尔曼滤波
入口参数：加速度、角速度
返回  值：无
**************************************************************************/
void Kalman_Filter(float Accel,float Gyro)		
{
	angle+=(Gyro - Q_bias) * dt; //先验估计
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

	Pdot[1]=-PP[1][1];
	Pdot[2]=-PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - angle;	//zk-先验估计
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	angle	+= K_0 * Angle_err;	 //后验估计
	Q_bias	+= K_1 * Angle_err;	 //后验估计
	angle_dot   = Gyro - Q_bias;	 //输出值(后验估计)的微分=角速度
}



void bsp_mpu_GetAngle(void)
{
	float Accx = 0, Accy = 0, Accz = 0;
	float Gyrox = 0, Gyroy = 0, Gyroz = 0;
	float PitchAcc = 0, RollAcc = 0, YawAcc = 0;
//	mpu_dmp_get_data(&Car.MPU.Pitch, &Car.MPU.Roll, &Car.MPU.Yaw);
	bsp_mpu_ReadAcc(&Car.MPU.Accx, &Car.MPU.Accy, &Car.MPU.Accz);
	bsp_mpu_ReadGyro(&Car.MPU.Gyrox, &Car.MPU.Gyroy, &Car.MPU.Gyroz);
	
	Accx = Car.MPU.Accx;
	Accy = Car.MPU.Accy;
	Accz = Car.MPU.Accz;
	
	Gyrox = Car.MPU.Gyrox - MPU_GYROX_ZERO;
	Gyroy = Car.MPU.Gyroy - MPU_GYROY_ZERO;
	Gyroz = Car.MPU.Gyroz - MPU_GYROZ_ZERO;
	
	if(Gyrox > 32768) Gyrox -= 65536;
	if(Gyroy > 32768) Gyroy -= 65536;
	if(Gyroz > 32768) Gyroz -= 65536;
	
	Gyrox = Gyrox/16.4;
	Gyroy = Gyroy/16.4;
	Gyroz = Gyroz/16.4;
	
	PitchAcc = atan2(Accx, Accz) * 180 / PI;
//	RollAcc = atan2(Accy, Accz) * 180 / PI;	
//		filter_KalmanFilter(&PitchKalman, Gyroy, PitchAcc);
	Kalman_Filter(PitchAcc, -Gyroy);

//	filter_KalmanFilter(&RollKalman, Gyrox, RollAcc);
	
	Car.MPU.Pitch = -angle;
	Car.MPU.Yaw = -PitchKalman.X[0];
}

/********************************************  END OF FILE  *******************************************/

