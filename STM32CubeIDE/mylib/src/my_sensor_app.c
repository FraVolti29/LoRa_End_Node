/*
 * my_lorawan.h
 *
 *  Created on: Aug 9, 2024
 *      Author: Roberto La Rosa
 */

#include "my_sensor_app.h"

#include "sht40ad1b.h"
#include "stm32wlxx_nucleo_bus.h"

#include "stts22h.h"
#include "sths34pf80.h"
#include "lis2du12.h"

uint8_t sht40_id = 0;		// SHT40 Sensor ID
uint8_t stts22h_id = 0;		// STTS22H Sensor ID
uint8_t sths34pf_id = 0;	// STHS34PF Sensor ID
uint8_t lis2du12_id = 0;	// LIS2DU12 Sensor ID

static SHT40AD1B_Object_t sht40_obj;
SHT40AD1B_IO_t            sht40_io_ctx;

static STTS22H_Object_t stts22h_obj;
STTS22H_IO_t stts22h_io_ctx;

static STHS34PF80_Object_t sths34pf80_obj;
STHS34PF80_IO_t sths34pf80_io_ctx;

float SHT40_Temp;
float SHT40_Humid;
float STTS22H_Temp;

uint32_t my_SHT40_Temp = 0;
uint32_t my_SHT40_Humid = 0;
uint32_t my_STTS22H_Temp = 0;

int16_t sths34pf80_motion;
int16_t sths34pf80_presence;

float sths34pf80_Obj_temp;
uint32_t my_sths34pf80_Obj_temp = 0;

LIS2DU12_Object_t	lis2du12_obj;
LIS2DU12_IO_t		lis2du12_io_ctx;
LIS2DU12_Axes_t 	lis2du12_acc_data;

uint8_t my_SHT40AD1B_TEMP_Status = 0;
uint8_t my_SHT40AD1B_HUM_Status = 0;

uint8_t my_STTS22H_TEMP_Status = 0;

int32_t my_STHS34PF80_GetMotionData_Status = 0;
int32_t my_STHS34PF80_GetPresenceData_Status =  0;
int32_t my_STHS34PF80_GetObjectTemperature_Status = 0;

uint8_t my_SHT40AD1B_Id_Status = 0;
uint8_t my_STTS22H_Id_Status = 0;
uint8_t my_STHS34PF80_Id_Status = 0;

volatile uint8_t my_Sensors_Data_Ready = 0;
volatile uint8_t my_Sensors_Id_Status = 0;

void My_SHT40_Sensor_Init(void)
{
	BSP_I2C2_DeInit();		// Deinitialize I2C2

	uint8_t id = 0;

	volatile int32_t                 ret = BSP_ERROR_NONE;

	sht40_io_ctx.BusType		= 0; /* I2C */
	sht40_io_ctx.Address		= 0x89;
	sht40_io_ctx.Init			= BSP_I2C2_Init;
	sht40_io_ctx.DeInit			= BSP_I2C2_DeInit;
	sht40_io_ctx.Read			= BSP_I2C2_Recv;
	sht40_io_ctx.Write			= BSP_I2C2_Send;
	sht40_io_ctx.GetTick		= BSP_GetTick;
	sht40_io_ctx.Delay			= My_HAL_Delay;

	if (SHT40AD1B_RegisterBusIO(&sht40_obj, &sht40_io_ctx) != SHT40AD1B_OK)
	{
		ret = BSP_ERROR_UNKNOWN_COMPONENT;
	}
	else if (SHT40AD1B_ReadID(&sht40_obj, &id) != SHT40AD1B_OK)
	{
		ret = BSP_ERROR_UNKNOWN_COMPONENT;
	}
	else
	{
		ret = BSP_ERROR_UNKNOWN_COMPONENT;
	}

	SHT40AD1B_TEMP_Enable(&sht40_obj);
	SHT40AD1B_HUM_Enable(&sht40_obj);

	if (id == 0) // Check if the SHT40 IC is Detected
	{
		my_SHT40AD1B_Id_Status = 1;
	}
	sht40_id = id;
}

void My_SHT40_Sensor_Data_Read(void)
{
	/**** Get Temperature & Humidity from SHT40 ****/
	my_SHT40AD1B_TEMP_Status = SHT40AD1B_TEMP_GetTemperature(&sht40_obj, &SHT40_Temp);
	my_SHT40AD1B_HUM_Status = SHT40AD1B_HUM_GetHumidity(&sht40_obj, &SHT40_Humid);

	my_SHT40_Temp = (uint32_t)(10*SHT40_Temp);
	my_SHT40_Humid = (uint32_t)(10*SHT40_Humid);
}

void My_STTS22H_Sensor_Init(void)
{
	BSP_I2C2_DeInit();		// Deinitialize I2C2

	uint8_t id = 0;

	volatile int32_t                 ret = BSP_ERROR_NONE;

	stts22h_io_ctx.BusType		= 0; /* I2C */
	stts22h_io_ctx.Address		= STTS22H_I2C_ADD_L;
	stts22h_io_ctx.Init			= BSP_I2C2_Init;
	stts22h_io_ctx.DeInit		= BSP_I2C2_DeInit;
	stts22h_io_ctx.ReadReg		= BSP_I2C2_ReadReg;
	stts22h_io_ctx.WriteReg		= BSP_I2C2_WriteReg;
	stts22h_io_ctx.GetTick		= BSP_GetTick;
	stts22h_io_ctx.Delay		= My_HAL_Delay;

	if (STTS22H_RegisterBusIO(&stts22h_obj, &stts22h_io_ctx) != STTS22H_OK)
	{
		ret = BSP_ERROR_UNKNOWN_COMPONENT;
	}
	else if (STTS22H_ReadID(&stts22h_obj, &id) != STTS22H_OK)
	{
		ret = BSP_ERROR_UNKNOWN_COMPONENT;
	}
	else
	{
		ret = BSP_ERROR_UNKNOWN_COMPONENT;
	}

	STTS22H_DeInit(&stts22h_obj);
	STTS22H_Init(&stts22h_obj);
	STTS22H_TEMP_Enable(&stts22h_obj);

	if (id == 0xA0) // The IC is Detected
	{
		my_STTS22H_TEMP_Status = 1;
	}
	stts22h_id = id;
}

void My_STTS22H_Sensor_Data_Read(void)
{
	/**** Get Temperature from STTS22H ****/
	my_STTS22H_TEMP_Status = STTS22H_TEMP_GetTemperature(&stts22h_obj, &STTS22H_Temp);
	my_STTS22H_Temp = (uint32_t)(STTS22H_Temp);
}

void My_STHD34PF80_Sensor_Init(void)
{
	BSP_I2C2_DeInit();		// Deinitialize I2C2

	uint8_t id = 0;

	volatile int32_t                 ret = BSP_ERROR_NONE;

	sths34pf80_io_ctx.BusType	= 0; /* I2C */
	sths34pf80_io_ctx.Address	= STHS34PF80_I2C_ADD;
	sths34pf80_io_ctx.Init		= BSP_I2C2_Init;
	sths34pf80_io_ctx.DeInit	= BSP_I2C2_DeInit;
	sths34pf80_io_ctx.ReadReg	= BSP_I2C2_ReadReg;
	sths34pf80_io_ctx.WriteReg	= BSP_I2C2_WriteReg;
	sths34pf80_io_ctx.GetTick	= BSP_GetTick;
	sths34pf80_io_ctx.Delay		= My_HAL_Delay;

	if (STHS34PF80_RegisterBusIO(&sths34pf80_obj, &sths34pf80_io_ctx) != STTS22H_OK)
	{
		ret = BSP_ERROR_UNKNOWN_COMPONENT;
	}
	else if (STHS34PF80_ReadID(&sths34pf80_obj, &id) != STTS22H_OK)
	{
		ret = BSP_ERROR_UNKNOWN_COMPONENT;
	}
	else
	{
		ret = BSP_ERROR_UNKNOWN_COMPONENT;
	}

	STHS34PF80_DeInit(&sths34pf80_obj);
	STHS34PF80_Init(&sths34pf80_obj);
	STHS34PF80_TEMP_Enable(&sths34pf80_obj);

	if (id == 0xD3) // 0xD3 = 211
	{
		my_STHS34PF80_Id_Status = 1;
	}
	sths34pf_id = id;
}
void My_STHD34PF80_Sensor_Data_Read(void)
{
	/**** Get Data from STHS34PF80 ****/
	my_STHS34PF80_GetMotionData_Status = STHS34PF80_GetMotionData(&sths34pf80_obj, &sths34pf80_motion);
	my_STHS34PF80_GetPresenceData_Status =  STHS34PF80_GetPresenceData(&sths34pf80_obj, &sths34pf80_presence);
	my_STHS34PF80_GetObjectTemperature_Status = STHS34PF80_GetObjectTemperature(&sths34pf80_obj, &sths34pf80_Obj_temp);
}

void My_LIS2DU12_Sensor_Init(void)
{
	BSP_I2C1_DeInit();		// Deinitialize I2C1

	uint8_t id = 0;

	int32_t                 ret = BSP_ERROR_NONE;

	/*************************** LIS2DU12  Accelerometer Init************/

	/* Configure the pressure driver */
	lis2du12_io_ctx.BusType     = 0; /* I2C */
	lis2du12_io_ctx.Address     = LIS2DU12_I2C_ADD_H;
	lis2du12_io_ctx.Init        = BSP_I2C1_Init;
	lis2du12_io_ctx.DeInit      = BSP_I2C1_DeInit;
	lis2du12_io_ctx.ReadReg     = BSP_I2C1_ReadReg;
	lis2du12_io_ctx.WriteReg    = BSP_I2C1_WriteReg;
	lis2du12_io_ctx.GetTick     = BSP_GetTick;
	lis2du12_io_ctx.Delay       = My_HAL_Delay;

	if (LIS2DU12_RegisterBusIO(&lis2du12_obj, &lis2du12_io_ctx) != LIS2DU12_OK)
	{
		ret = BSP_ERROR_UNKNOWN_COMPONENT;
	}
	else if (LIS2DU12_ReadID(&lis2du12_obj, &id) != LIS2DU12_OK)
	{
		ret = BSP_ERROR_UNKNOWN_COMPONENT;
	}
	else
	{
		ret = BSP_ERROR_UNKNOWN_COMPONENT;
	}

	LIS2DU12_Init(&lis2du12_obj);
	LIS2DU12_ACC_Enable(&lis2du12_obj);

	lis2du12_id = id;
	/*************************************************************************/
}
void My_LIS2DU12_Sensor_Data_Read(void)
{

	/**** Get Accelerometer reading from LIS2DU12 ****/
	LIS2DU12_ACC_GetAxes(&lis2du12_obj, &lis2du12_acc_data);

}

void My_HAL_Delay(__IO uint32_t Delay)
{
	/* TIMER_IF can be based on other counter the SysTick e.g. RTC */
	/* USER CODE BEGIN HAL_Delay_1 */

	/* USER CODE END HAL_Delay_1 */
	TIMER_IF_DelayMs(Delay);
	/* USER CODE BEGIN HAL_Delay_2 */

	/* USER CODE END HAL_Delay_2 */
}


#if 0
void Temp_Humidity_Sensor_Init(void)
{

	/*************************** SHT40  TEMP & HUMIDTY SENSOR Init************/
	int32_t                 ret = BSP_ERROR_NONE;

	/* Configure the pressure driver */
	sht40_io_ctx.BusType	= 0; /* I2C */
	sht40_io_ctx.Address	= 0x89;
	sht40_io_ctx.Init		= BSP_I2C2_Init;
	sht40_io_ctx.DeInit		= BSP_I2C2_DeInit;
	sht40_io_ctx.Read		= BSP_I2C2_Recv;
	sht40_io_ctx.Write		= BSP_I2C2_Send;
	sht40_io_ctx.GetTick	= BSP_GetTick;
	sht40_io_ctx.Delay		= My_HAL_Delay;

	if (SHT40AD1B_RegisterBusIO(&sht40_obj, &sht40_io_ctx) != SHT40AD1B_OK)
	{
		ret = BSP_ERROR_UNKNOWN_COMPONENT;
	}
	else if (SHT40AD1B_ReadID(&sht40_obj, &id) != SHT40AD1B_OK)
	{
		ret = BSP_ERROR_UNKNOWN_COMPONENT;
	}
	else
	{
		ret = BSP_ERROR_UNKNOWN_COMPONENT;
	}

	SHT40AD1B_TEMP_Enable(&sht40_obj);
	SHT40AD1B_HUM_Enable(&sht40_obj);

	if (id == 0) // Check if the SHT40 IC is Detected
	{
		my_SHT40AD1B_Id_Status = 1;
	}
	sht40_id = id;

	/*************************************************************************/


	/*************************** STTS22H  TEMP SENSOR Init************/
	/* Configure the pressure driver */
	stts22h_io_ctx.BusType     = 0; /* I2C */
	stts22h_io_ctx.Address     = STTS22H_I2C_ADD_L;
	stts22h_io_ctx.Init        = BSP_I2C2_Init;
	stts22h_io_ctx.DeInit      = BSP_I2C2_DeInit;
	stts22h_io_ctx.ReadReg     =    BSP_I2C2_ReadReg;
	stts22h_io_ctx.WriteReg    =    BSP_I2C2_WriteReg;
	stts22h_io_ctx.GetTick     = BSP_GetTick;
	stts22h_io_ctx.Delay =       My_HAL_Delay;

	if (STTS22H_RegisterBusIO(&stts22h_obj, &stts22h_io_ctx) != STTS22H_OK)
	{
		ret = BSP_ERROR_UNKNOWN_COMPONENT;
	}
	else if (STTS22H_ReadID(&stts22h_obj, &id) != STTS22H_OK)
	{
		ret = BSP_ERROR_UNKNOWN_COMPONENT;
	}
	else
	{
		ret = BSP_ERROR_UNKNOWN_COMPONENT;
	}

	STTS22H_TEMP_Enable(&stts22h_obj);
	//  SHT40AD1B_TEMP_Enable(&sht40_obj);

	if (id == 0xA0) // The IC is Detected
	{
		my_STTS22H_TEMP_Status = 1;
	}

	stts22h_id = id;

	/***********************************************************************/



	/*************************** STHS34PF80 IR & Presence Detection Sensor Init************/
	/* Configure the pressure driver */
	sths34pf80_io_ctx.BusType     = 0; /* I2C */
	sths34pf80_io_ctx.Address     = STHS34PF80_I2C_ADD;
	sths34pf80_io_ctx.Init        = BSP_I2C2_Init;
	sths34pf80_io_ctx.DeInit      = BSP_I2C2_DeInit;
	sths34pf80_io_ctx.ReadReg     =    BSP_I2C2_ReadReg;
	sths34pf80_io_ctx.WriteReg    =    BSP_I2C2_WriteReg;
	sths34pf80_io_ctx.GetTick     = BSP_GetTick;
	sths34pf80_io_ctx.Delay 	  = My_HAL_Delay;

	if (STHS34PF80_RegisterBusIO(&sths34pf80_obj, &sths34pf80_io_ctx) != STTS22H_OK)
	{
		ret = BSP_ERROR_UNKNOWN_COMPONENT;
	}
	else if (STHS34PF80_ReadID(&sths34pf80_obj, &id) != STTS22H_OK)
	{
		ret = BSP_ERROR_UNKNOWN_COMPONENT;
	}
	else
	{
		ret = BSP_ERROR_UNKNOWN_COMPONENT;
	}

	STHS34PF80_TEMP_Enable(&sths34pf80_obj);

	if (id == 0xD3) // The IC is Detected
	{
		my_STHS34PF80_Id_Status = 1;
	}
	sths34pf_id = id;
	/************************************************************************/

	if (my_STTS22H_Id_Status && my_SHT40AD1B_Id_Status && my_STHS34PF80_Id_Status == 1)
	{
		my_Sensors_Id_Status = 1;
	}

}

void Accelerometer_Sensor_Init(void)
{
	/**
	 * @brief LIS2DU12 Initialization Function
	 * @param None
	 * @retval None
	 */

	int32_t                 ret = BSP_ERROR_NONE;
	//	BSP_I2C1_Init();

	/*************************** LIS2DU12  Accelerometer Init************/

	/* Configure the pressure driver */
	lis2du12_io_ctx.BusType     = 0; /* I2C */
	lis2du12_io_ctx.Address     = LIS2DU12_I2C_ADD_L;
	lis2du12_io_ctx.Init        = BSP_I2C1_Init;
	lis2du12_io_ctx.DeInit      = BSP_I2C1_DeInit;
	lis2du12_io_ctx.ReadReg     =    BSP_I2C1_ReadReg;
	lis2du12_io_ctx.WriteReg    =    BSP_I2C1_WriteReg;
	lis2du12_io_ctx.GetTick     = BSP_GetTick;
	lis2du12_io_ctx.Delay =       HAL_Delay;

	if (LIS2DU12_RegisterBusIO(&lis2du12_obj, &lis2du12_io_ctx) != LIS2DU12_OK)
	{
		ret = BSP_ERROR_UNKNOWN_COMPONENT;
	}
	else if (LIS2DU12_ReadID(&lis2du12_obj, &id) != LIS2DU12_OK)
	{
		ret = BSP_ERROR_UNKNOWN_COMPONENT;
	}
	else
	{
		ret = BSP_ERROR_UNKNOWN_COMPONENT;
	}

	LIS2DU12_Init(&lis2du12_obj);
	LIS2DU12_ACC_Enable(&lis2du12_obj);
	/*************************************************************************/
}

void Get_Temp_Humidity_data(void)
{
	/**
	 * @brief Get data from SHT40 ,STTS22H,STHS34PF80  sensors
	 * @param None
	 * @retval None
	 */

	/**** Get Temperature & Humidity from SHT40 ****/
	my_SHT40AD1B_TEMP_Status = SHT40AD1B_TEMP_GetTemperature(&sht40_obj, &SHT40_Temp);
	my_SHT40AD1B_HUM_Status = SHT40AD1B_HUM_GetHumidity(&sht40_obj, &SHT40_Humid);

	my_SHT40_Temp = (uint32_t)(SHT40_Temp);
	my_SHT40_Humid = (uint32_t)(SHT40_Humid);


	/**** Get Temperature from STTS22H ****/
	my_STTS22H_TEMP_Status = STTS22H_TEMP_GetTemperature(&stts22h_obj, &STTS22H_Temp);
	my_STTS22H_Temp = (uint32_t)(STTS22H_Temp);

	/**** Get Temperature from STHS34PF80 ****/
	my_STHS34PF80_GetMotionData_Status = STHS34PF80_GetMotionData(&sths34pf80_obj, &sths34pf80_motion);
	my_STHS34PF80_GetPresenceData_Status =  STHS34PF80_GetPresenceData(&sths34pf80_obj, &sths34pf80_presence);
	my_STHS34PF80_GetObjectTemperature_Status = STHS34PF80_GetObjectTemperature(&sths34pf80_obj, &sths34pf80_Obj_temp);

	my_Sensors_Data_Ready =
			my_SHT40AD1B_TEMP_Status &&
			my_SHT40AD1B_HUM_Status &&
			my_STTS22H_TEMP_Status &&
			my_STHS34PF80_GetMotionData_Status &&
			my_STHS34PF80_GetPresenceData_Status &&
			my_STHS34PF80_GetObjectTemperature_Status;
}

void Get_Accelerometer_data(void)
{
	/**
	 * @brief Get data from LIS2DU12  sensors
	 * @param None
	 * @retval None
	 */

	/**** Get Accelerometer reading from LIS2DU12 ****/
	LIS2DU12_ACC_GetAxes(&lis2du12_obj, &lis2du12_acc_data);

}
#endif



#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
