# include "bsp_mpu.h"
# include "FreescaleCar.h"
# include "app_filter.h"
# include "math.h"
# include "bsp_i2c.h"

/*  MPU IIC���ƽṹ��  */
IIC_TypeDef mpu_iic;


/*
*********************************************************************************************************
*                      MPU_SDA_HIGH                    
*
* Description: ����SDA���ŵ�ƽΪ��
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
* Description: ����SDA���ŵ�ƽΪ��
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
* Description: ����SDA����Ϊ���ģʽ
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
* Description: ����SDA����Ϊ����ģʽ
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
* Description: ����SCL���ŵ�ƽΪ��
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
* Description: ����SCL���ŵ�ƽΪ��
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
* Description: ��ȡSDA���ŵ�ƽֵ
*             
* Arguments  : None.
*
* Reutrn     : SDA���ŵ�ƽֵ
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
* Description: ��ʼ��MPU��IIC
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
* Description: MPU��ʼ������
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
	bsp_tim_DelayMs(100);
	bsp_mpu_WriteByte(MPU_PWR_MGMT1_REG, 0x00);
	bsp_mpu_SetGyroFsr(3);
	bsp_mpu_SetAccelFsr(0);
	bsp_mpu_SetRate(100);
	bsp_mpu_WriteByte(MPU_USER_CTRL_REG, 0x00);
	bsp_mpu_WriteByte(MPU_FIFO_EN_REG, 0xff);
	bsp_mpu_WriteByte(MPU_INTBP_CFG_REG, 0x80);
	bsp_mpu_WriteByte(MPU_INT_EN_REG, 0x00);
	
	res = bsp_mpu_ReadByte(MPU_DEVICE_ID_REG);
	
	/*  ��ȡMPU��ַ���ж��Ƿ��ʼ���ɹ�  */
	if(MPU_ADDR == res)
	{
		bsp_mpu_WriteByte(MPU_PWR_MGMT1_REG, 0x01);
		bsp_mpu_WriteByte(MPU_PWR_MGMT2_REG, 0x00);
		bsp_mpu_SetRate(50);
	}else
		return 1;
	
	while(mpu_dmp_init());	/*  �ȴ�DMP���ʼ�����  */
	
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
#define acc_x_zero -300.0//???????
#define acc_y_zero -850.0
#define acc_z_zero 700.0
#define mdps_lsb 62.5
#define PI 3.14159265
float g_speed_gyro;//
float g_angle_acc;//

extern float angle, angle_dot; 

short Accx[10], Accy[10], Accz[10];
short Gyrox[10], Gyroy[10], Gyroz[10];
uint8_t write = 0;
void bsp_mpu_DataProcess(void)
{
//	if(mpu_dmp_get_data(&Car.MPU.Pitch, &Car.MPU.Roll, &Car.MPU.Yaw) == 0)
//	{
//		bsp_mpu_ReadAcc(&Car.MPU.Accx, &Car.MPU.Accy, &Car.MPU.Accz);
//	bsp_mpu_ReadGyro(&Car.MPU.Gryox, &Car.MPU.Gryoy, &Car.MPU.Gryoz);
//	}
	if(Car.MPU.Gryoy>32768)  Car.MPU.Gryoy-=65536;                       //��������ת��  Ҳ��ͨ��shortǿ������ת��
	if(Car.MPU.Gryoz>32768)  Car.MPU.Gryoz-=65536;                       //��������ת��
	if(Car.MPU.Accx>32768) Car.MPU.Accx-=65536;                      //��������ת��
	if(Car.MPU.Accz>32768) Car.MPU.Accz-=65536;   

	Car.MPU.Accy=atan2(Car.MPU.Accx,Car.MPU.Accz)*180/PI;
	Car.MPU.Gryoy=Car.MPU.Gryoy/16.4;
	
//	Kalman_Filter(Car.MPU.Accy, - Car.MPU.Gryoy);
//	
//	Car.MPU.Pitch = angle;
//	}
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
* Description: ����MPU�ĵ�ͨ�˲���
*             
* Arguments  : 1> lpf: ��ͨ�˲���ֵ
*
* Reutrn     : 1> ��ȡ�����ļĴ���ֵ
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


/********************************************  END OF FILE  *******************************************/
