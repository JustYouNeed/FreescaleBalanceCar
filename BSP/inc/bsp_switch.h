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
	
# ifndef __BSP_SWITCH_H
# define __BSP_SWITCH_H

# include "headfile.h"

# define BIT1_PIN			I1
# define BIT2_PIN			I2
# define BIT3_PIN			E2
# define BIT4_PIN			E3



void bsp_switch_Config(void);

uint8_t bsp_switch_GetValue(void);
# endif

/********************************************  END OF FILE  *******************************************/

