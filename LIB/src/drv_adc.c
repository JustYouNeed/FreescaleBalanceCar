/**
  *******************************************************************************************************
  * File Name: drv_adc.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-3-1
  * Brief: KEA128оƬADC�ײ���������
  *******************************************************************************************************
  * History
  *		1.Data: 2018-3-1
	*     Author: Vector
	*     Mod: �����ļ�,���ӻ�������
  *
  *******************************************************************************************************
  */
	

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "drv_adc.h"


/*
  *******************************************************************************************************
  *                              LOCAL VARIABLE
  *******************************************************************************************************
*/

static uint16_t 	ADC_Result[24] = {0};		/*  �ݴ�ADCת�����  */
static uint8_t  	ADC_Count = 0;					/*  ʹ�õ�ADCͨ����  */
static uint32_t   ADC_Order[24] = {0};		/*  ADCͨ��˳��,������ȡ����ADCת�����  */


/*
*********************************************************************************************************
*                              drv_adc_Init            
*
* Description: ��ʼ��ADC����
*             
* Arguments  : 1> ADC_Initstruct: ADC��ʼ���ṹ��ָ��
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void drv_adc_Init(ADC_InitTypeDef *ADC_InitStruct)
{
	uint32_t posbit = 0, channel = 0;
	uint8_t reg = 0;
	uint8_t i = 0, cnt = 0;
	
	//drv_rcc_ClockCmd(RCC_PeriphClock_ADC, ENABLE);  /*  ����ADCʱ��  */
	SIM->SCGC |= SIM_SCGC_ADC_MASK;
	
	ADC->SC3 = 0;
	ADC->APCTL1 |= ADC_InitStruct->ADC_Channel;     /*  ����ADC����ͨ��  */
	ADC->SC3 |= ADC_InitStruct->ADC_Resolution;			/*  ����ADC����λ��  */
	ADC->SC3 |= ADC_InitStruct->ADC_Prescaler;			/*  ʱ�ӷ�Ƶ  */
	ADC->SC3 |= ADC_InitStruct->ADC_ClockSource;		/*  ʱ��Դ  */
	ADC->SC2 |= ADC_InitStruct->ADC_RefSource;			/*  �ο���ѹԴ  */
	
	if(ADC_InitStruct->ADC_ContinuousConvMode == ENABLE)	/*  �������������ת��ģʽ  */
		reg |= ADC_SC1_ADCO_MASK;
	
	if(ADC_InitStruct->ADC_IRQCmd == ENABLE)		/*  ���ʹ�����ж�  */
	{
		reg |= ADC_SC1_AIEN_MASK;
		NVIC_EnableIRQ(ADC_IRQn);
	}
	
	if(ADC_InitStruct->ADC_ScanConvMode != DISABLE)		/*  ���������ɨ��ģʽ  */
	{
		ADC->SC4 |= ADC_SC4_ASCANE_MASK;
	}
	
	/*  ���������FIFO�������,����Ҫ����Ҫ������ADCͨ������д��FIFO,ֱ��д��,ϵͳ�Ż���������  */
	channel = ADC_InitStruct->ADC_Channel;
	ADC->SC4 |= (uint8_t)(ADC_InitStruct->ADC_ChannelCount - 1); /*  ����FIFO���  */
	
	for(i = 0; i < 16; i++)	/*  һλһλ���  */
	{
		posbit = (uint32_t)((channel >> i) & 0x01);
		if(posbit)		/*  ��������˸�ͨ��  */
		{
			ADC->SC1 = (ADC_SC1_ADCH(i) | reg);
			
			ADC_Order[ADC_Count++] = (uint32_t)(1 << i);
			cnt++;
		}
		
		/*  �Ѿ��������������ͨ��  */
		if(cnt >= ADC_InitStruct->ADC_ChannelCount) break;	
	}
	
	/*  ��������˲���VSS  */
	if(channel & ADC_Channel_VSS)	
	{
		ADC->SC1 = (ADC_SC1_ADCH(16) | reg);
		ADC_Order[ADC_Count++] = ADC_Channel_VSS;
	}
	
	/*  �������ڲ��¶Ȳ���  */
	if(channel & ADC_Channel_Temperature)
	{
		ADC->SC1 = (ADC_SC1_ADCH(22) | reg);
		ADC_Order[ADC_Count++] = ADC_Channel_Temperature;
	}
}



/*
*********************************************************************************************************
*                                ADC0_IRQHandler          
*
* Description: ADC�жϺ���,����ת�����
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void ADC0_IRQHandler(void)
{
	uint8_t i = 0;
	
	for(; i < ADC_Count; i++)
	{
		ADC_Result[i] = ADC->R;
	}
}


/*
*********************************************************************************************************
*                         drv_adc_GetMultiADCResult                 
*
* Description: ��ȡ���ADCת�����
*             
* Arguments  : 1> ADCBuff: ADC���������ָ��
*
* Reutrn     : None.
*
* Note(s)    : ע��,�û������Ĵ�С����С����ʹ�õ�����
*********************************************************************************************************
*/
void drv_adc_GetMultiADCResult(uint16_t *ADCBuff)
{
	uint8_t i = 0;
	
	for(; i < ADC_Count; i++)		
	{
		ADCBuff[i] = ADC_Result[i];
	}
}


/*
*********************************************************************************************************
*                              drv_adc_GetSingleADCResult            
*
* Description: ��ͨ��ADC����ʱ��ȡ����ͨ���Ĳ������
*             
* Arguments  : 1> ADC_Channel: ADCͨ��
*
* Reutrn     : 1> 0xffff:ͨ������
*              2> ��Ӧͨ����ת�����
*
* Note(s)    : �ú���ֻ����ADC�����˶�ͨ������ʱ������
*********************************************************************************************************
*/
uint16_t drv_adc_GetSingleADCResult(uint32_t ADC_Channel)
{
	uint16_t cnt = 0;
	
	for(; cnt < ADC_Count; cnt++)		/*  �õ�ͨ��  */
	{
		if(ADC_Order[cnt] == ADC_Channel) break;
	}
	if(cnt >= ADC_Count) return 0xffff;	/*  δ�ҵ�ͨ��  */
	
	return ADC_Result[cnt];
}

/*
*********************************************************************************************************
*                             drv_adc_ConvOnce             
*
* Description: ת��һ��ͨ����ADCһ��
*             
* Arguments  : 1> ADC_Channel: Ҫת����ADCͨ��
*              2> ADC_Resolution: ADCת��λ��
*
* Reutrn     : 1> ADCת�����
*
* Note(s)    : �ú���ֻ��ת��һ��ͨ���Ľ��,�������ͨ���ж��,��ת����λͨ��
*********************************************************************************************************
*/
uint16_t drv_adc_ConvOnce(uint32_t ADC_Channel, uint8_t ADC_Resolution)
{
	uint16_t channel = 0;
	uint16_t result = 0;
	
	for(; channel < 16; channel++)		/*  �õ�ͨ��  */
	{
		if((ADC_Channel >> channel) & 0x01) break;
	}
	
	if(ADC_Channel & ADC_Channel_VSS) channel = 16;
	if(ADC_Channel & ADC_Channel_Temperature) channel = 22;
	
	ADC->SC3 |= ADC_Resolution;			/*  ����ת��λ��  */
	ADC->SC3 |= ADC_SC3_ADIV(0);		/*  ����Ƶ  */
	ADC->SC3 |= ADC_SC3_ADICLK(0);	/*  ʹ������ʱ��  */

	ADC->SC2 |= ADC_SC2_REFSEL(0);	/*  �ⲿ�ο���ѹ  */
	
	ADC->SC1 = ADC_SC1_ADCH(channel);				/*  ����ת��  */
	while(!(ADC->SC1 & ADC_SC1_COCO_MASK));	/*  �ȴ�ת�����  */
	result = ADC->R;
	ADC_SC1 &= ~ADC_SC1_COCO_MASK;	/*  ���ת����ɱ�־  */
	return (result);       /*  ����ת�����  */
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
void drv_adc_StartConv(void)
{
//	uint8_t i = 0;
//	
//	ADC->SC1 = ADC->SC1;
//	while(!(ADC->SC1 & ADC_SC1_COCO_MASK));
//	
//	for(; i < ADC_Count; i++)
//	{
//		ADC_Result[i] = ADC->R;
//	}
	uint8_t reg = ADC->SC1;
	
	reg |= 1 << 6;
	
	ADC->SC1 = reg;
}

/********************************************  END OF FILE  *******************************************/