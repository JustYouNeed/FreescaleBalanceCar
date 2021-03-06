# ifndef __BSP_H
# define __BSP_H

# include "headfile.h"
# include "bsp_led.h"
# include "bsp_uart.h"
# include "bsp_timer.h"
# include "bsp_key.h"
# include "bsp_oled.h"
# include "bsp_flash.h"
# include "bsp_motor.h"
# include "bsp_encoder.h"
# include "bsp_sensor.h"
# include "bsp_switch.h"

# define PID_PARA_FLASH_ADDR			254
# define PID_PARA_LENGTH					60

# define SENSOR_PARA_FLASH_ADDR		253
# define SENSOR_PARA_LENGTH				60

# define CAR_PARA_FLASH_ADDR			252


void bsp_Config(void);

# endif

