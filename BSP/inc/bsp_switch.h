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

# define BIT1_PIN			G7
# define BIT2_PIN			G6
# define BIT3_PIN			F1
# define BIT4_PIN			F0



void bsp_switch_Config(void);

uint8_t bsp_switch_GetValue(void);
# endif

/********************************************  END OF FILE  *******************************************/

