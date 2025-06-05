/*
 * my_sensor_app.h
 *
 *  Created on: Aug 9, 2024
 *      Author: Roberto La Rosa
 */

#ifndef __SENSOR_APP_H
#define __SENSOR_APP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wlxx_hal.h"
#include "stm32wlxx_nucleo.h"
#include <stdio.h>

#include "timer_if.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/


/* USER CODE BEGIN Private defines */
void Get_Accelerometer_data(void);
void Get_Temp_Humidity_data(void);
void Accelerometer_Sensor_Init(void);
void Temp_Humidity_Sensor_Init(void);
void My_HAL_Delay(__IO uint32_t Delay);

void My_SHT40_Sensor_Init(void);
void My_SHT40_Sensor_Data_Read(void);

void My_STTS22H_Sensor_Init(void);
void My_STTS22H_Sensor_Data_Read(void);

void My_STHD34PF80_Sensor_Init(void);
void My_STHD34PF80_Sensor_Data_Read(void);

void My_LIS2DU12_Sensor_Init(void);
void My_LIS2DU12_Sensor_Data_Read(void);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
