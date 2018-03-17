/**
  *******************************************************************************************************
  * File Name: 
  * Author: 
  * Version: 
  * Date: 
  * Brief: 
  *******************************************************************************************************
  * History
  *
  *
  *******************************************************************************************************
  */	
	
# ifndef __APP_PID_H
# define __APP_PID_H

/*
	*******************************************************************************************************
	*                              INCLUDE FILES
	*******************************************************************************************************
*/
# include "bsp.h"
# include "common.h"

typedef struct
{
	float Balance_Kp, Balance_Ki, Balance_Kd;			/*  平衡环PID  */
	float Velocity_Kp, Velocity_Ki, Velocity_Kd;	/*  速度环PID  */
	float Turn_Kp, Turn_Ki, Turn_Kd;							/*  转向环PID  */
	float Error, ErrSum;																	/*  积分  */
}PID_TypeDef;

extern PID_TypeDef PID;

void pid_ParaInit(void);
void pid_ReadPara(void);
void pid_StorePara(void);

# endif
	
	
/********************************************  END OF FILE  *******************************************/
	

