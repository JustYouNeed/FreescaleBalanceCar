/**
  *******************************************************************************************************
  * File Name: app_filter.h
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-3-2
  * Brief: ���ļ������˸����˲�����
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-2
	*			Mod: �����ļ�
  *
  *******************************************************************************************************
  */	

# ifndef __APP_FILTER_H
# define __APP_FILTER_H
/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "bsp.h"

/*  һά�������˲��ṹ��  */
typedef struct 
{
	double Q;
	double R;
	double P;
	double Kg;
	double Output;
}Kalman1Dim_TypeDef;


typedef struct
{
	float X[2];			/*  ��������״̬X,������̬������˵,X[0]������̬��  */
	int16_t Gyro;			/*  ���Ž��ٶ�  */
	float A[2][2];		/*  AΪϵͳΪk-1��kʱ�̵�״̬ת�ƾ���  */
	float B[2];			/*  BΪϵͳ����  */
	float P[2][2];		/*  PΪϵͳЭ����  */
	float Q[2][2];		/*  QΪϵͳ����  */
	float R;					/*  ��������  */
	float Kg[2];			/*  KgΪϵͳ����  */	
	float dt;				/*  �˲�������ʱ��  */
}Kalman_TypeDef;

void filter_SildingAverage(uint16_t Array[], uint16_t *Average, uint16_t Length);
void filter_Kalman1Dim_Init(Kalman1Dim_TypeDef *Kalman_struct, double Q, double R);
void filter_Kalman1Dim(Kalman1Dim_TypeDef *Kalam_Struct, double input);
void filter_KanlmanInit(Kalman_TypeDef *Kalman);
void filter_KalmanFilter(Kalman_TypeDef *Kalman, float Gyro, float AccAngle);
# endif

/********************************************  END OF FILE  *******************************************/

