/*
 * my_lorawan.c
 *
 *  Created on: Aug 9, 2024
 *      Author: Roberto La Rosa
 */
#include "my_lorawan.h"
#include "radio.h"
#include "stm32_seq.h"
#include "stm32_timer.h"
#include "main.h"
#include "gpio.h"
#include "app_lorawan.h"
#include "platform.h"
#include "sys_debug.h"
#include "adc_if.h"

#include "stm32wlxx_hal_def.h"
#include "stm32wlxx_hal_lptim.h"

#include "rtc.h"
#include "lptim.h"
#include "comp.h"
#include <stdlib.h>

#include "app_x-cube-ai.h"
#include "ai_platform_interface.h"
// Private Variables Begin

//static ai_buffer* ai_input;
//static ai_buffer* ai_output;
//static ai_handle network_appl_temp = AI_HANDLE_NULL;

uint8_t my_ESD_Level = 0;
uint8_t My_EBK_status = 0;

uint8_t my_RTS_cnt = 0;
uint8_t my_ESS_cnt = 0;
uint8_t my_SMS_cnt = 0;

uint8_t my_ems_index = 0;

uint8_t my_rtc_A_ems_flag = 0;
uint8_t my_rtc_B_ems_flag = 0;

uint8_t my_Check_ESD = 0;
uint8_t my_Check_Sensors = 0;

uint16_t my_Trise_time_start = 0;
uint16_t my_Trise_time_stop = 0;
uint16_t my_Trise_time = 0;

uint8_t my_Comparator_flag = 1;
uint8_t my_rm_failure_flag = 0;
uint8_t my_sm_failure_flag = 0;
uint16_t my_Tau_Resistance = 0;
uint16_t my_Start_Timer = 0;
uint16_t my_lptim_value = 0;
uint16_t my_ems_buffer[MY_DIM_EMS_BUFFER];

// Private Variables End

PWR_PVDTypeDef sConfigPVD;

My_Mode_State_td my_Previous_State = CSS;
My_Mode_State_td my_Current_State = CSS;
My_Mode_State_td my_Next_State = EHS;

My_Cmode_td my_Cmode = DCM;

#if STEVAL_HARVEST1

#if MY_SoilSensor
// Calibration values
//#define AIR_VALUE 2890.0f
//#define WATER_VALUE 1516.5f
#define AIR_VALUE 1879.37f
#define WATER_VALUE 1309.0f

float my_SoilSensor = 0;
#endif

#if MY_SHT40
float my_SHT40_Temp_Float = 0;
float my_SHT40_Humid_Float = 0;
uint32_t my_SHT40_Temp = 0;
uint32_t my_SHT40_Humid = 0;
uint32_t my_SHT40_Flag = 0;					// Initialize SHT40 Sensor Presence Flag
#endif

#if MY_STTS22H
uint32_t my_STTS22H_Sensor_Flag = 0;		// Initialize STTS22H Sensor Presence Flag
uint32_t my_STTS22H_Temp = 0;				// Initialize STTS22H Temperature
#endif

#if MY_STHS34PF80
uint8_t my_STHS34PF80_Sensor_Flag = 0;		// Initialize STHS34PF80 Sensor Presence Flag
uint8_t my_STHS34PF80_Motion = 0;     		// Initialize STHS34PF80 Motion Flag
uint8_t my_STHS34PF80_Presence = 0;    		// Initialize STHS34PF80 Presence Flag
uint8_t my_STHS34PF80_Tamb_Shock = 0;   	// Initialize STHS34PF80 Ambient Temperature Shock Flag

uint32_t my_STHS34PF80_Obj_Temp = 0;      	// Initialize Object Temperature Value
uint32_t my_STHS34PF80_Amb_Temp = 0;    	// Initialize Ambient Temperature Value

uint16_t my_STHS34PF80_Presence_Data = 0;	// Initialize STHS34PF80 Presence Data
uint16_t my_STHS34PF80_Motion_Data = 0;		// Initialize STHS34PF80 Motion Data
#endif

#if MY_LIS2DU12
uint8_t my_LIS2DU12_Sensor_Flag = 0;		// Initialize LIS2DU12 Sensor Presence Flag
uint16_t my_LIS2DU12_acc_x = 0;				// Initialize LIS2DU12 Acc X
uint16_t my_LIS2DU12_acc_y = 0;				// Initialize LIS2DU12 Acc Y
uint16_t my_LIS2DU12_acc_z = 0;				// Initialize LIS2DU12 Acc Z
#endif

#endif

RTC_TimeTypeDef sTime = {0};
RTC_DateTypeDef sDate = {0};
RTC_AlarmTypeDef sAlarm = {0};

extern uint8_t my_SystemClock_Config_done;
extern uint8_t my_MX_GPIO_Init_done;
extern uint8_t my_SubghzApp_Init_done;
extern uint8_t my_RTS_cnt;
extern LPTIM_HandleTypeDef hlptim1;
extern LPTIM_HandleTypeDef hlptim2;
extern COMP_HandleTypeDef hcomp1;
extern COMP_HandleTypeDef hcomp2;
extern RTC_HandleTypeDef hrtc;

/* CallBack Section Begin */

void HAL_PWR_PVDCallback(void)	// PVD CallBack
{
	if(my_SystemClock_Config_done == 0)											// Check if the PS Detection has been executed
	{
		my_Next_State = RTS;
	}
	else
	{
		switch(my_Current_State)
		{
		case EHS:  																// EHS = Energy Harvesting State
		{
			My_EHS_to_EMS_Timer(STOP_TIMER);									// Stop EHS_to_EMS Timer

			switch(my_Next_State)
			{

			case RTS:															// RTS = Radio Transmission State
			{
				if (my_SubghzApp_Init_done == 1) 								// Check if Radio Initialization is done
				{
					if (my_RTS_cnt == 0)
					{
						My_Trise_Meas();										// Measure the Trise Time
						My_SMW_Timer(STOP_TIMER);								// Stop the SMW Timer
					}

					UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_RTF), CFG_SEQ_Prio_0);	// SendTxData
				}
				break;
			}

			case ESS:															// ESS = Energy Storage State
			{
				UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_ESF), CFG_SEQ_Prio_0);  	// My_ESF -- ESF = Energy Storage Function
				break;
			}

			case SMS:															// SMS = Sensor Measurement State
			{
				UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SMF), CFG_SEQ_Prio_0);  	// My_SMF -- SMF = Sensors Measurement Function
				break;
			}
#if AI
			case AIS:
			{
				UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_AIF), CFG_SEQ_Prio_0);  	// My_AIF -- AIF = Artificial Intelligence Function
				break;
			}
#endif

			default:
			{
				my_Next_State = SMS;											// Set Next State as SMS
				UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SMF), CFG_SEQ_Prio_0);  	// My_SMF -- SMF = Sensors Measurement Function
				break;
			}
			}

			break;
		}

		case ESS:  														  		// ESS = Energy Storage State
		{
			switch (my_Cmode)
			{

			case DCM:  															// Discontinuous Charge Mode
			{
				my_Current_State = EHS;  										// Set Current State as EHS: 	ESS --> EHS
				My_DCM_to_CCM_Timer(STOP_TIMER);  								// Stop the DCM to CCM Timer
				UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_EHF), CFG_SEQ_Prio_0);		// My_EHF - Back to Harvest
				break;
			}

			case EOC:  															// End of Charge Mode
			{
				if (my_Check_ESD == 0)
				{
					my_Cmode = EOC;  											// End of Charge Mode has been met
				}

				My_ESW_CCM_Timer(STOP_TIMER);  									// Stop the Energy Storage Window Timer
				My_EHS_to_EMS_Timer(STOP_TIMER);  								// Stop the EHS to EMS Timer

				my_Current_State = EHS;  										// Set Current State as EHS: 	ESS --> EHS

#if MY_SM_FEATURE_ENABLE
				my_Next_State = SMS;											// Set Next State as RTS:	 	ESS --> EHS --> SMS
#else
				my_Next_State = RTS;  											// Set Next State as RTS:	 	ESS --> EHS --> RTS
#endif
				UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_EHF), CFG_SEQ_Prio_0);  	// My_EHF - Back to Harvest
				break;
			}

			default:
			{
				my_Current_State = EHS;  										// Set Current State as EHS
				my_Next_State = SMS;											// Set Next State as SMS
				UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SMF), CFG_SEQ_Prio_0);  	// My_SMF -- SMF = Sensors Measurement Function
				break;
			}
			}

			break;
		}

		case SMS:
		{

#if MY_LIS2DU12 && STEVAL_HARVEST1
			My_VDD_to_VDDS1_Switch(Switch_OPEN);								// unBias the Sensors: Open the switch between VDD and VDDS1
#endif

#if MY_SHT40 || MY_STHS34PF80 || MY_STTS22H && STEVAL_HARVEST1
			My_VDD_to_VDDS2_Switch(Switch_OPEN);								// unBias the Sensors: Open the switch between VDD and VDDS2
#endif

			if (!((my_Cmode == DCM) && (my_Check_ESD == 0)))
			{
				Radio.Sleep();													// Switch the Radio Off
			}

			my_Current_State = EHS;												// Set Current State as EHS

			if ((my_Cmode == EOC && HAL_LPTIM_ReadCounter(&hlptim1) > (T_ESW + T_SMW)) ||
					(my_Cmode != EOC && HAL_LPTIM_ReadCounter(&hlptim1) > T_SMW) ||
					(my_SMS_cnt > SMS_MAX))
			{
				my_Trise_time_start = HAL_LPTIM_ReadCounter(&hlptim1);  		// Measure Starting Trise Time
#if !AI
				my_Next_State = RTS;  											// SMS --> RTS
#endif
#if AI
				my_Next_State = AIS;  											// SMS --> AIS
#endif
			}

			UTIL_SEQ_SetTask((1 <<CFG_SEQ_Task_EHF ), CFG_SEQ_Prio_0); 			// My_EHF
			break;
		}

		case EMS:  																// EMS = Energy Missing State
		{
			My_VDD_to_ES_Switch(Switch_OPEN);									// Open the Switch between Vdd and ES
			my_Cmode = EOC;  													// The Storage Element is in EOC since the PVD has been triggered with the VEOC threshold while in EMS.
			My_EMS_to_EDS_Timer(STOP_TIMER);  									// Stop the EMS to EDS Timer
			My_EMS_to_SMS_Timer(STOP_TIMER);  									// Stop the EMS to SMS Timer
			my_ems_index = 0;  													// Reset the index used to fill the buffer that contains the measurements performed during the ems state

#if MY_DM_FEATURE_ENABLE  														// If measurements during the night have been enabled
			my_Previous_State = EMS;
#endif

			my_Current_State = EHS;  											// Set Current State as EHS: 	EMS --> EHS
			my_Next_State = RTS;  												// Set Next State as RTS: 		EMS --> EHS --> RTS
			UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_EHF), CFG_SEQ_Prio_0);  		// My_EHF - Back to Harvest
			break;
		}

		case EDS:  																// EDS = Energy Detection State
		{
			My_VDD_to_ES_Switch(Switch_OPEN);  									// Open the Switch connected to the Storage element

			my_Cmode = DCM;  													// Assume that the Energy Storage Device (ESD) is discharged

			My_EDS_Timer(STOP_TIMER);  											// Stop the EDS Timer
			My_EMS_to_SMS_Timer(STOP_TIMER);									// Stop the EMS_to_SMS Timer

			my_rtc_A_ems_flag = 0;
			my_rtc_B_ems_flag = 0;

			my_RTS_cnt = 0;  													// Resets the transmissions counter
			my_ems_index = 0;  													// Reset the index used to fill the buffer that contains the measurements performed during the ems state

#if MY_DM_FEATURE_ENABLE
			my_Previous_State = EMS;
#endif

			UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_RTF), CFG_SEQ_Prio_0);  		// SendTxData
			break;
		}
		default:
		{
			my_Current_State = EHS;  											// Set Current State as EHS
			my_Next_State = SMS;												// Set Next State as SMS
			UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SMF), CFG_SEQ_Prio_0);  		// My_SMF -- SMF = Sensors Measurement Function
			break;
		}

		}
	}
}

void HAL_LPTIM_AutoReloadMatchCallback(LPTIM_HandleTypeDef *hlptim)	// LPTIM Callback
{
	if(hlptim->Instance == LPTIM1)
	{
		switch(my_Current_State)
		{
		case EHS: 															// EHS = Energy Harvesting State
		{
			switch (my_Next_State)
			{
			case ESS:
			{

#if MY_SM_FEATURE_ENABLE
				my_Next_State = SMS; 										// 	ESS --> SMS
#else
				my_Next_State = RTS; 										// ESS -->  RTS
#endif

				My_DCM_to_CCM_Timer(STOP_TIMER);							// Stop the DCM to CCM Timer
				My_ESW_CCM_Timer(STOP_TIMER);								// Stop ESW Timer

				UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_EHF), CFG_SEQ_Prio_0);	// My_EHF -- Back to Harvest
				break;
			}

			default:
			{
				my_Current_State = EHS;  									// Set Current State as EHS
				my_Next_State = SMS;										// Set Next State as SMS
				UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SMF), CFG_SEQ_Prio_0);  // My_SMF -- SMF = Sensors Measurement Function
				break;
			}
			}

			break;
		}

		case ESS:															// ESS = Energy Storage State
		{
			if (my_Cmode != DCM)
			{
#if MY_SM_FEATURE_ENABLE
				my_Next_State = SMS;										// Set Next State as SMS: ESS --> EHS --> SMS
#else
				my_Next_State = RTS;										// Set Next State as RTS: ESS --> EHS --> RTS
#endif

				my_Current_State = EHS;										// Set Current State as EHS: ESS --> EHS

				My_DCM_to_CCM_Timer(STOP_TIMER);							// Stop the DCM to CCM Timer
				My_ESW_CCM_Timer(STOP_TIMER);									// Stop ESW Timer

				UTIL_SEQ_SetTask((1 <<CFG_SEQ_Task_EHF ), CFG_SEQ_Prio_0);	// My_EHF -- Back to Harvest
			}
			break;
		}

		case EMS:															// EMS = Energy Missing State
		{
			My_EDS_Timer(STOP_TIMER); 										// Stop the Energy Detection Timer
			my_rm_failure_flag = 1; 										// Resistance Measurement in EMS State has failed because the LPTIM1 Interrupt has been triggered earlier than the COMP Interrupt
			break;
		}

		case EDS:															// EDS = Energy Detection State
		{
			My_EMS_to_EDS_Timer(START_TIMER); 								// Start the Timer for the Energy Presence Check (EDC)
			My_VDD_to_ES_Switch(Switch_CLOSED);								// Close the switch Between Vdd and ES Start Supplying from the Energy Storage Device (ESD)

#if MY_LIS2DU12 && STEVAL_HARVEST1
			My_VDD_to_VDDS1_Switch(Switch_OPEN);							// unBias the Sensors: Open the switch between VDD and VDDS1
#endif

#if MY_SHT40 && MY_STHS34PF80 && MY_STTS22H && STEVAL_HARVEST1
			My_VDD_to_VDDS2_Switch(Switch_OPEN);							// unBias the Sensors: Open the switch between VDD and VDDS2
#endif

			my_Current_State = EMS; 										// Set Current State as EMS: EDS --> EMS
			UTIL_SEQ_SetTask((1 <<CFG_SEQ_Task_EHF), CFG_SEQ_Prio_0);		// My_EHF -- Back to Harvest
			break;
		}

		default:
		{
			my_Current_State = EHS;  										// Set Current State as EHS
			my_Next_State = SMS;											// Set Next State as SMS
			UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SMF), CFG_SEQ_Prio_0);  	// My_SMF -- SMF = Sensors Measurement Function
			break;
		}
		}
	}
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)			// RTC Alarm A CallBack
{

	switch(my_Current_State)
	{

	case EMS: 														// EMS = Energy Missing State
	{
		My_EMS_to_SMS_Timer(STOP_TIMER);							// Stop EMS to SMS Timer
		my_rtc_A_ems_flag = 0;

#if MY_RM_FEATURE_ENABLE
		UTIL_SEQ_SetTask((1 <<CFG_SEQ_Task_RMF), CFG_SEQ_Prio_0); 	// My_RMF
#endif

		my_Current_State = EMS;										// Set Current State as EMS
		UTIL_SEQ_SetTask((1 <<CFG_SEQ_Task_EHF ), CFG_SEQ_Prio_0); 	// My_EHF
		break;
	}

	case SMS:
	{

#if MY_LIS2DU12 && STEVAL_HARVEST1
		My_VDD_to_VDDS1_Switch(Switch_OPEN);						// unBias the Sensors: Open the switch between VDD and VDDS1
#endif

#if (MY_SHT40 || MY_STHS34PF80 || MY_STTS22H) && STEVAL_HARVEST1
		My_VDD_to_VDDS2_Switch(Switch_OPEN);						// unBias the Sensors: Open the switch between VDD and VDDS2
#endif

		my_Current_State = EHS;										// Set Current State as EHS
		UTIL_SEQ_SetTask((1 <<CFG_SEQ_Task_EHF ), CFG_SEQ_Prio_0); 	// My_EHF
		break;
	}

	default:
	{
#if MY_TX_OTAA
		UTIL_TIMER_IRQ_Handler();
#endif
		break;
	}
	}
}

void HAL_RTCEx_AlarmBEventCallback(RTC_HandleTypeDef *hrtc)	// RTC Alarm B CallBack
{
	switch (my_Current_State)
	{

	case ESS: 															// ESS = Energy Storage State

		/* The Time Measured by the RTC Alarm B Timer has gone beyond T_DCM_CCM
		 * The system switches from Discontinuous Charge Mode (DCM) to Continuous Charge Mode (CCM)
		 */

	{
		My_Set_PVD(VPS_CHECK, PWR_PVD_MODE_NORMAL);						// Configures PVD at the Highest Voltage and Normal Mode
		My_HAL_Delay(100);												// Add Delay to settle PVD

		if (__HAL_PWR_GET_FLAG(PWR_FLAG_PVDO) == 0)						// If VDD is > VPS_CHECK and Internal Reference is Ready
		{
			my_Check_ESD = 1;
			My_VDD_to_ES_Switch(Switch_OPEN);							// Open the VDD to ES Switch
			my_Current_State = SMS;										// Set Current State as SMS
			my_Next_State = SMS;										// Set Next State as SMS
			UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SMF), CFG_SEQ_Prio_0); 	// My_SMF = Sensors Measurement Function
		}
		else
		{
			my_Cmode = CCM;												// Switch to Continuous Charge Mode
			My_ESW_DCM_Timer(STOP_TIMER);								// Stop ESW DCM Timer
			My_ESW_CCM_Timer(START_TIMER);								// Start ESW CCM Timer

			my_Current_State = ESS;										// System keeps Staying in Energy Storage State (ESS)
			UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_ESF), CFG_SEQ_Prio_0); 	// My_ESF = Energy Storage Function
		}

		My_DCM_to_CCM_Timer(STOP_TIMER);								// Stop DCM to CCM Timer

		break;
	}

	/* If RTC ALARM B is triggered during EHS: EHS --> EMS to manage Energy absence*/
	case EHS: 														// EHS = Energy Harvesting State
	{
		My_EHS_to_EMS_Timer(STOP_TIMER); 							// Stop EHS_to_EMS Timer
		HAL_RTC_DeactivateAlarm(hrtc, RTC_ALARM_A);					// Stop RTC A Alarm

		my_rtc_B_ems_flag = 0;
		my_rtc_A_ems_flag = 0;

		my_Current_State = EMS;										// Set Current State as EMS: EHS --> EMS
		UTIL_SEQ_SetTask((1 <<CFG_SEQ_Task_EHF ), CFG_SEQ_Prio_0);  // My_EHF -- Back to Harvest
		break;
	}

	/*If the Alarm B has been Triggered during the EMS a EDS is performed */
	case EMS: 														// EMS = Energy missing State
	{
		My_VDD_to_ES_Switch(Switch_OPEN);							// Open The Switch Beteween Vdd and ES

		HAL_RTC_DeactivateAlarm(hrtc, RTC_ALARM_A);					// Deactivate Alarm A
		HAL_RTC_DeactivateAlarm(hrtc, RTC_ALARM_B);					// Deactivate Alarm B

		my_rtc_B_ems_flag = 0;
		my_rtc_A_ems_flag = 0;

		my_Current_State = EDS; 									// Set Current State as EDS: EMS --> EDS
		UTIL_SEQ_SetTask((1 <<CFG_SEQ_Task_EHF ), CFG_SEQ_Prio_0);	// My_EHF
		break;
	}

	default:
	{

#if MY_TX_ABP
		my_Current_State = EHS;  									// Set Current State as EHS
		my_Next_State = SMS;										// Set Next State as SMS
		UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SMF), CFG_SEQ_Prio_0);  // My_SMF -- SMF = Sensors Measurement Function
#endif
		break;
	}

	}
}

/* CallBack Section End */
/*my rtc init*/
void My_RTC_Init(void)
{
#if MY_RTC_INIT
	HAL_NVIC_DisableIRQ(RTC_Alarm_IRQn);								// Disable Interrupt for RTC Alarms (A and B) Interrupt

	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDate = {0};
	RTC_AlarmTypeDef sAlarm = {0};

	/* USER CODE BEGIN RTC_Init 1 */
#undef CFG_RTCCLK_DIVIDER_CONF								// redefine the parameters to have the clock at 1 Hz (to count in 1 second steps)
#undef CFG_RTC_WUCKSEL_DIVIDER
#undef CFG_RTCCLK_DIV
#undef CFG_RTC_ASYNCH_PRESCALER
#undef CFG_RTC_SYNCH_PRESCALER

#define CFG_RTCCLK_DIVIDER_CONF 16
#define CFG_RTC_WUCKSEL_DIVIDER (0)
#define CFG_RTCCLK_DIV              CFG_RTCCLK_DIVIDER_CONF
#define CFG_RTC_ASYNCH_PRESCALER    (CFG_RTCCLK_DIV - 1)
#define CFG_RTC_SYNCH_PRESCALER     (DIVR( LSE_VALUE, (CFG_RTC_ASYNCH_PRESCALER+1) ) - 1 )
	/* USER CODE END RTC_Init 1 */

	/** Initialize RTC Only
	 */
	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = CFG_RTC_ASYNCH_PRESCALER;
	hrtc.Init.SynchPrediv = CFG_RTC_SYNCH_PRESCALER;
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
	if (HAL_RTC_Init(&hrtc) != HAL_OK)
	{
		Error_Handler();
	}

	/* USER CODE BEGIN Check_RTC_BKUP */

	/* USER CODE END Check_RTC_BKUP */

	/** Initialize RTC and set the Time and Date
	 */
	sTime.Hours = 0;
	sTime.Minutes = 0;
	sTime.Seconds = 0;
	sTime.SubSeconds = 0x0;
	sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
	{
		Error_Handler();
	}
	sDate.WeekDay = RTC_WEEKDAY_MONDAY;
	sDate.Month = RTC_MONTH_JANUARY;
	sDate.Date = 1;
	sDate.Year = 0;
	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
	{
		Error_Handler();
	}

	/** Enable the Alarm A
	 */
	sAlarm.AlarmTime.Hours = 0;
	sAlarm.AlarmTime.Minutes = 0;
	sAlarm.AlarmTime.Seconds = 0;
	sAlarm.AlarmTime.SubSeconds = 0;
	sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
	sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
	sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
	sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
	sAlarm.AlarmDateWeekDay = 1;
	sAlarm.Alarm = RTC_ALARM_A;
	if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_NVIC_DisableIRQ(RTC_Alarm_IRQn);								// Disable Interrupt for RTC Alarms (A and B) Interrupt
#endif
}

/* FSM Function Begin */

void My_EHF(void)	// EHF = Energy Harvesting Function
{
	/*
	 * The EHF Function manages the Energy Harvesting feature of the system
	 */

#if !(MY_DEBUGGER_ENABLED || MY_LPTIM_Monitor || MY_RTCB_Monitor)
	My_Set_All_GPIO_To_Analog_Mode();										// Set all GPIO in Analog Mode
	My_Interrupts_Manager(NVIC_CLEAR); 										// Clear all Pending Interrupts
#endif

#if MY_ES_FEATURE_ENABLE
	My_VDD_to_ES_Switch(Switch_OPEN);										// Open VDD to ES Switch
#endif

	switch (my_Current_State)
	{
	case EHS:  																// EHS = Energy Harvesting State
	{
		My_EHS_to_EMS_Timer(START_TIMER);  									// Start the EHS to EMS timer

		switch (my_Next_State)
		{

		case ESS:
		{
			My_Enter_Stop2_Mode_WFI(VESS_HIGH, PWR_PVD_MODE_IT_RISING); 	// Set Enter Stop2 Mode and WFI
			break;
		}

		case SMS:
		{
			My_Enter_Stop2_Mode_WFI(VSMS_HIGH, PWR_PVD_MODE_IT_RISING); 	// Set Enter Stop2 Mode and WFI
			break;
		}
#if AI
		case AIS:
		{
			My_Enter_Stop2_Mode_WFI(VAIS_HIGH, PWR_PVD_MODE_IT_RISING); 	// Set Enter Stop2 Mode and WFI
			break;
		}
#endif

		default:
		{
			My_Enter_Stop2_Mode_WFI(VRTS, PWR_PVD_MODE_IT_RISING); 			// Set Enter Stop2 Mode and WFI
			break;
		}
		}

		break;
	}

	case RTS:  																// RTS = Radio Transmission State
	{
#if MY_ES_FEATURE_ENABLE  													// If the Energy Storage Feature is enabled
		if (++my_RTS_cnt >= N_LORA_TX)  									// If the number of transmissions has reached the threshold
		{
			if (my_Cmode != EOC)											// If the Energy Storage Device (ESD) is NOT in End of Charge (EOC)
			{
				my_Current_State = EHS;  									// RTS --> EHS
				my_Next_State = ESS;  										// RTS --> EHS --> ESS
			}
			else  															// If Energy Storage Device (ESD) is in End of Charge (EOC)
			{
				My_VDD_to_ES_Switch(Switch_OPEN);  							// Open the Vdd to ES Switch
				my_Current_State = EHS;  									// RTS --> EHS

#if MY_SM_FEATURE_ENABLE  													// If the Sensor Measurements Feature is enabled
				my_Next_State = SMS;										// RTS --> EHS --> SMS
#else
				my_Next_State = RTS;  										// RTS --> EHS --> RTS
#endif
			}
			my_RTS_cnt = 0;  												// Reset the transmissions counter
		}
		else  																// If the number of Radio transmissions is lower than N_LORA_TX
		{
			my_Current_State = EHS;  										// RTS --> EHS
			my_Next_State = RTS;  											// RTS --> EHS --> RTS
		}
#else  																		// If ES Feature is not enabled
		my_Current_State = EHS;  											// RTS --> EHS

#if MY_RM_FEATURE_ENABLE
		if (++my_RTS_cnt >= N_LORA_TX)  									// If the number of transmissions has reached the threshold
		{
			my_Next_State = RMS;											// RTS --> RMS
			my_Current_State = EHS;											// RTS --> RMS --> EHS
			my_RTS_cnt = 0;
		}
#else
		my_Next_State = RTS;  												// Set Following State as RTS
#endif
#endif

		My_EHS_to_EMS_Timer(START_TIMER);  									// Start the EHS to EMS Timer
		My_Enter_Stop2_Mode_WFI(VRTS, PWR_PVD_MODE_IT_RISING);  			// Enter Stop2 Power Mode and Wait for Interrupt
		break;
	}

	case ESS:
	{
		if (my_Next_State == SMS)
		{
			My_Enter_Stop2_Mode_WFI(VSMS_HIGH, PWR_PVD_MODE_IT_RISING);  	// Enter Stop2 Power Mode and Wait for Interrupt
		}
		break;
	}

	case SMS:
	{
		My_EHS_to_EMS_Timer(START_TIMER);  									// Start the EHS to EMS Timer
		My_Enter_Stop2_Mode_WFI(VSMS_HIGH, PWR_PVD_MODE_IT_RISING);  		// Enter Stop2 Power Mode and Wait for Interrupt
		break;
	}

	case EMS:  																// EMS = Energy Missing State
	{
		My_ESW_DCM_Timer(STOP_TIMER);  										// Stop the ESW Timer
		My_ESW_CCM_Timer(STOP_TIMER);  										// Stop the ESW Timer

#if MY_SM_FEATURE_ENABLE

#if MY_LIS2DU12 && STEVAL_HARVEST1
		My_VDD_to_VDDS1_Switch(Switch_OPEN);								// unBias the Sensors: Open the switch between VDD and VDDS1
#endif

#if (MY_SHT40 || MY_STHS34PF80 || MY_STTS22H) && STEVAL_HARVEST1
		My_VDD_to_VDDS2_Switch(Switch_OPEN);								// unBias the Sensors: Open the switch between VDD and VDDS2
#endif

#endif

		My_VDD_to_ES_Switch(Switch_CLOSED);  								// The system is supplied by the Energy Storage Device (ESD)

		My_PVD_Delay();														// 2 sec delay
		My_PVD_Delay();														// 2 sec delay

		if (my_rtc_B_ems_flag == 0)  										// Flag the occurrence of EDS
		{
			My_EMS_to_EDS_Timer(START_TIMER);  								// Start an Energy Presence Check
			my_rtc_B_ems_flag = 1;
		}

		if (my_rtc_A_ems_flag == 0)  										// Flag the occurrence of ems Measurement
		{
			My_EMS_to_SMS_Timer(START_TIMER);  								// Start the timer to move from EMS to SMS
			my_rtc_A_ems_flag = 1;
		}

		My_Enter_Stop2_Mode_WFI(PVD_EMS, PWR_PVD_MODE_IT_RISING);  			// Enter Stop2 Power Mode and Wait for Interrupt
		break;
	}

	case EDS:  																// EDS = Energy Detection State
	{
		My_EDS_Timer(START_TIMER);  										// Start the EDS Timer
		My_Enter_Stop2_Mode_WFI(VRTS, PWR_PVD_MODE_IT_RISING);				// Enter Stop2 Power Mode and Wait for Interrupt
		break;
	}

	default:
	{
		My_Enter_Stop2_Mode_WFI(VRTS, PWR_PVD_MODE_IT_RISING);				// Enter Stop2 Power Mode and Wait for Interrupt
		break;
	}
	}
}

void My_ESF(void)	// ESF = Energy Storage Function

/*
 * The ESF Function performs the actions executed during the Energy Storage State (ESS).
 * In the Energy Storage State (ESS) the additional Energy Storage Device (ESD) gets charged.
 */

{

#if !(MY_DEBUGGER_ENABLED || MY_LPTIM_Monitor || MY_RTCB_Monitor)
	My_Set_All_GPIO_To_Analog_Mode();									// Set all GPIO in Analog Mode
	My_Interrupts_Manager(NVIC_CLEAR); 									// Clear all Pending Interrupts
#endif

	my_Check_ESD = 0;													// Reset my_Check_ESD

	my_Current_State = ESS;												// Set Current State as Energy Storage State (ESS)

	switch (my_Cmode)
	{

	case DCM: 															// Discontinuous Charge Mode
	{
		My_Interrupts_Manager(NVIC_DISABLE);							// Disable all the interrupts
		HAL_NVIC_EnableIRQ(PVD_PVM_IRQn);								// Enable only interrupts for PVD During DCM

		if (HAL_LPTIM_ReadCounter(&hlptim1) > T_ESW || (my_ESS_cnt >= DCM_MAX))
		{
			My_ESW_DCM_Timer(STOP_TIMER);
			my_Next_State = SMS;
		}
		else
		{
			My_ESW_DCM_Timer(START_TIMER);								// Start the EWS DCM Timer
		}

		My_DCM_to_CCM_Timer(START_TIMER); 								// Start Timer to monitor DCM to CCM.
		My_VDD_to_ES_Switch(Switch_CLOSED);								// Close the Switch (High side of a GPIO) between Vdd and ES
		My_Enter_Stop2_Mode_WFI(VESS_LOW, PWR_PVD_MODE_IT_FALLING); 	// Set PVD to level VESS_LOW with Interrupt for Falling Vdd and Enter Stop2 Mode and Wait for Interrupt
		break;
	}

	case CCM: 															// Continuous Charge Mode
	{
		My_ESW_CCM_Timer(START_TIMER);									// Start the EWS CCM Timer
		My_VDD_to_ES_Switch(Switch_CLOSED);								// Close the Switch (High side of a GPIO) between Vdd and ES
		My_Set_PVD(VEOC, PWR_PVD_MODE_NORMAL); 							// Set PVD in Normal Mode and VEOC
		My_PVD_Delay(); 												// Delay to allow the PVD to Settle

		if (__HAL_PWR_GET_FLAG(PWR_FLAG_PVDO) == 0) 					// If the voltage across the Energy Storage Device (ESD) is higher than VEOC the System is in End of Charge (EOC)
		{
			my_Cmode = EOC; 											// Set Charging Mode in End of Charge (EOC)
			my_Current_State = EHS;										// Set Current State as Energy Harvesting State (EHS): ESS --> EHS
			My_EBK_status = 100;

#if MY_SM_FEATURE_ENABLE
			my_Next_State = SMS; 										// Set Next State as Radio Transmission State (RTS): ESS --> EHS --> SMS
#else
			my_Next_State = RTS; 										// Set Next State as Radio Transmission State (RTS): ESS --> EHS --> RTS
#endif
			My_VDD_to_ES_Switch(Switch_OPEN); 							// Stop Charging: Open the Switch between Vdd and ES

			My_ESW_CCM_Timer(STOP_TIMER);								// Stop the Energy Storage Window (ESW) Timer
			My_EHS_to_EMS_Timer(START_TIMER); 							// Start the EHS to EMS Timer (Detects if Energy to Harvest is Missing)

			My_Enter_Stop2_Mode_WFI(VRTS, PWR_PVD_MODE_IT_RISING); 		// Enter Stop 2 Power Mode and Wait for Interrupt
		}
		else
		{
			if (VEOC == V2P6)
			{
				My_Set_PVD(V2P2, PWR_PVD_MODE_NORMAL); 					// Set PVD in Normal Mode and VEOC
				My_PVD_Delay(); 										// Delay to allow the PVD to Settle

				if (__HAL_PWR_GET_FLAG(PWR_FLAG_PVDO) == 0) 			// Check if the voltage across the Energy Storage Device (ESD) is higher than V2P2
				{
					My_EBK_status = 33;
				}

				My_Set_PVD(V2P4, PWR_PVD_MODE_NORMAL); 					// Set PVD in Normal Mode and VEOC
				My_PVD_Delay();

				if (__HAL_PWR_GET_FLAG(PWR_FLAG_PVDO) == 0) 			// Check if the voltage across the Energy Storage Device (ESD) is higher than V2P4
				{
					My_EBK_status = 66;
				}

				My_Enter_Stop2_Mode_WFI(VEOC, PWR_PVD_MODE_IT_RISING);	// Enter Stop2 Mode and Wait for Interrupt and wait for the charging to be completed
			}

			if (VEOC == V2P8)
			{
				My_Set_PVD(V2P2, PWR_PVD_MODE_NORMAL); 					// Set PVD in Normal Mode and VEOC
				My_PVD_Delay(); 										// Delay to allow the PVD to Settle

				if (__HAL_PWR_GET_FLAG(PWR_FLAG_PVDO) == 0) 			// Check if the voltage across the Energy Storage Device (ESD) is higher than V2P2
				{
					My_EBK_status = 25;
				}

				My_Set_PVD(V2P4, PWR_PVD_MODE_NORMAL); 					// Set PVD in Normal Mode and VEOC
				My_PVD_Delay();

				if (__HAL_PWR_GET_FLAG(PWR_FLAG_PVDO) == 0) 			// Check if the voltage across the Energy Storage Device (ESD) is higher than V2P4
				{
					My_EBK_status = 50;
				}

				My_Set_PVD(V2P6, PWR_PVD_MODE_NORMAL); 					// Set PVD in Normal Mode and VEOC
				My_PVD_Delay();

				if (__HAL_PWR_GET_FLAG(PWR_FLAG_PVDO) == 0) 			// Check if the voltage across the Energy Storage Device (ESD) is higher than V2P6
				{
					My_EBK_status = 75;
				}

				My_Enter_Stop2_Mode_WFI(VEOC, PWR_PVD_MODE_IT_RISING);	// Enter Stop2 Mode and Wait for Interrupt and wait for the charging to be completed
			}
		}
		break;
	}

	default:
	{
		break;
	}
	}
}

void My_SMF(void)	// SMF = Sensor Measurement Function
{
	/*
	 * SMF = Sensor Measurement Function
	 * The SMF Function Manages the Sensors
	 */

#if !(MY_DEBUGGER_ENABLED || MY_LPTIM_Monitor || MY_RTCB_Monitor)
	My_Set_All_GPIO_To_Analog_Mode();								// Set all GPIO in Analog Mode
	My_Interrupts_Manager(NVIC_CLEAR); 								// Clear all Pending Interrupts
#endif

	HAL_NVIC_DisableIRQ(RTC_WKUP_IRQn);								// Disable Interrupt for RTC_WKUP
	HAL_NVIC_DisableIRQ(RTC_Alarm_IRQn);							// Disable Interrupt for RTC_Alarm_IRQn
	HAL_NVIC_DisableIRQ(LPTIM1_IRQn);								// Disable Interrupt for LPTIM1

	my_Current_State = SMS;											// Set Current State as SMS
	My_SMW_Timer(START_TIMER);										// Start Sensor Measurement Window Timer

	My_Set_PVD(VSMS_LOW, PWR_PVD_MODE_IT_FALLING);					// Set PVD

	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);	// Set Internal Voltage Regulator at 1.2 V

#if STEVAL_HARVEST1

#if MY_SoilSensor
//	GPIO_InitTypeDef GPIO_InitStruct = {0};
//	GPIO_InitStruct.Pin = GPIO_PIN_7;
//	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	MX_GPIO_Init();
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7, GPIO_PIN_SET);
	My_HAL_Delay(40);
	My_SoilSensor_Get_Data();
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

//	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#endif

#if MY_LIS2DU12 && STEVAL_HARVEST1
	My_VDD_to_VDDS1_Switch(Switch_CLOSED);							// Bias the Sensors: Close the switch between VDD and VDDS1 to Power Sensors
	My_HAL_Delay(10);												// Add a Delay
	My_I2C_Init(MY_I2C1);											// Initialize I2C1
#endif

#if (MY_SHT40 || MY_STHS34PF80 || MY_STTS22H) && STEVAL_HARVEST1
	My_VDD_to_VDDS2_Switch(Switch_CLOSED);							// Bias the Sensors: Close the switch between VDD and VDDS2 to Power Sensors
	My_HAL_Delay(10);												// Add a Delay
	My_I2C_Init(MY_I2C2);											// Initialize I2C2
#endif

#if MY_SHT40 && STEVAL_HARVEST1
	My_SHT40_Get_Data();											// SHT40 Get Data
#endif

#if MY_STTS22H && STEVAL_HARVEST1
	My_STTS22H_Get_Data();											// STTS22H Get Data
#endif

#if MY_STHS34PF80 && STEVAL_HARVEST1
	My_STHS34PF80_Get_Data();										// STHS34PF80 Get Data
#endif

#if MY_LIS2DU12 && STEVAL_HARVEST1
	My_LIS2DU12_Get_Data();											// LIS2DU12 Get Data
#endif

#if MY_LIS2DU12 && STEVAL_HARVEST1
	My_I2C_Deinit(MY_I2C1);											// Initialize I2C1
#endif

#if (MY_SHT40 || MY_STHS34PF80 || MY_STTS22H) && STEVAL_HARVEST1
	My_I2C_Deinit(MY_I2C2);											// DeInitialize I2C2
#endif

#endif

	if (my_Cmode == DCM && my_Check_ESD == 0)						// If in DCM Mode --> Charge the Energy Backup Device
	{
		My_VDD_to_ES_Switch(Switch_CLOSED);							// Charge the Energy Backup Device
	}
	else
	{
		Radio.Rx(1);												// Turn the Radio in RX Mode
	}

	My_Interrupts_Manager(NVIC_DISABLE);							// Disable and Clear All Pending Interrupts
	HAL_NVIC_EnableIRQ(PVD_PVM_IRQn);								// Enable Interrupt for PVD
	__WFI();														// Wait for Interrupt From PVD
}

void My_AIF(void)
{
	My_Set_PVD(VAIS_LOW, PWR_PVD_MODE_IT_FALLING);					// Set PVD
	//My_PVD_Delay();
	My_SMW_Timer(STOP_TIMER);
	MX_X_CUBE_AI_Process();
	UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_RTF), CFG_SEQ_Prio_0);	// SendTxData
}

/* FSM Function End */


/* Timer Section Begin */

void My_ESW_DCM_Timer(My_Timer_td mode)
{
	/* This Function Starts and Stops the LPTIM1 timer to measure the time T_ESW that defines the Energy Storage Window (ESW) in DCM */

	if ((mode == START_TIMER) && (my_ESS_cnt++ == 0))	// Start the Energy Storage Window (ESW) Timer
	{
		HAL_LPTIM_Counter_Start(&hlptim1, (0XFFFF));	// Start the lptim1 timer used for the Energy Storage Window (ESW)

#if MY_LPTIM_Monitor
		My_LPTIM1_Monitor(Switch_CLOSED);
#endif
	}

	if (mode == STOP_TIMER) 							// Stop the Energy Storage Window (ESW) Timer
	{
		my_ESS_cnt = 0;									// Reset ESS Counter
		HAL_LPTIM_Counter_Stop(&hlptim1); 				// Stop the lptim1 timer used for the Energy Storage Window (ESW)

#if MY_LPTIM_Monitor
		My_LPTIM1_Monitor(Switch_OPEN);
#endif
	}
}

void My_ESW_CCM_Timer(My_Timer_td mode)
{
	/* This Function Starts and Stops the LPTIM1 timer to measure the time T_ESW that defines the Energy Storage Window (ESW) in CCM */

	if ((mode == START_TIMER) && (my_ESS_cnt++ == 0))	// Start the Energy Storage Window (ESW) Timer
	{
		HAL_LPTIM_Counter_Start_IT(&hlptim1, T_ESW);

#if MY_LPTIM_Monitor
		My_LPTIM1_Monitor(Switch_CLOSED);
#endif

	}

	if (mode == STOP_TIMER) 							// Stop the Energy Storage Window (ESW) Timer
	{
		my_ESS_cnt = 0;									// Reset ESS Counter
		HAL_LPTIM_Counter_Stop_IT(&hlptim1); 			// Stop the lptim1 timer used for the Energy Storage Window (ESW)

#if MY_LPTIM_Monitor
		My_LPTIM1_Monitor(Switch_OPEN);
#endif
	}
}

void My_SMW_Timer(My_Timer_td mode)
{
	/* This Function Starts and Stops the LPTIM1 timer to measure the time T_SMW that defines the Sensor Measure Window (SMW) */

	if ((mode == START_TIMER) && (my_SMS_cnt++ == 0))	// Start the Sensor Measurement Window (SMW) Timer
	{
		HAL_LPTIM_Counter_Start(&hlptim1, (0XFFFF));	// Start the lptim1 timer used for the Sensor Measurement Window (SMW)

#if MY_LPTIM_Monitor
		My_LPTIM1_Monitor(Switch_CLOSED);
#endif
	}

	if (mode == STOP_TIMER) 							// Stop the Sensor Measurement Window (SMW) Timer
	{
		my_SMS_cnt = 0;									// Reset my_SMS_cnt
		HAL_LPTIM_Counter_Stop(&hlptim1); 				// Stop the lptim1 timer used for the Sensor Measurement Window (SMW)

#if MY_LPTIM_Monitor
		My_LPTIM1_Monitor(Switch_OPEN);
#endif
	}
}

void My_EDS_Timer(My_Timer_td mode)
{
	/* This Function Starts and Stops the LPTIM1 timer to measure the time T_EDS that defines the Energy Detection State time Window */

	if (mode == START_TIMER) // Start the Energy Detection State (EDS) Timer
	{
		HAL_LPTIM_Counter_Start_IT(&hlptim1, T_EDS);

#if MY_LPTIM_Monitor
		My_LPTIM1_Monitor(Switch_CLOSED);
#endif
	}

	if (mode == STOP_TIMER)	// Stop the Energy Detection State (EDS) Timer
	{
		HAL_LPTIM_Counter_Stop_IT(&hlptim1);

#if MY_LPTIM_Monitor
		My_LPTIM1_Monitor(Switch_OPEN);
#endif
	}
}

void My_EHS_to_EMS_Timer(My_Timer_td mode)
{
	/*
	 * This Function Starts and Stops the EHS to EMS Timer
	 * It starts an RCT Alarm B Timer to detect, if the energy to harvest is missing while the System is in Energy Harvesting State (EHS).
	 * If the Energy to Harvest is missing for a time longer than T_EHS_EMS the System switches from the Energy Harvesting State (EHS) into the Energy Missing State (EMS).
	 */

	if (mode == START_TIMER)
	{
		HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_B);	// Deactivate the RTC alarm B

		My_Set_RTC_Alarm_B	// Start EHS to EMS Timer
		(
				T_EHS_EMS_hours,
				T_EHS_EMS_minutes,
				T_EHS_EMS_seconds,
				T_EHS_EMS_subseconds
		);
#if MY_RTCB_Monitor
		My_RTCB_Monitor(Switch_CLOSED);
#endif
	}

	if (mode == STOP_TIMER)	// Stop EHS to EMS Timer
	{
		HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_B);	// Deactivate the RTC alarm B
#if MY_RTCB_Monitor
		My_RTCB_Monitor(Switch_OPEN);
#endif
	}

}

void My_DCM_to_CCM_Timer(My_Timer_td mode)
{
	/*
	 * This Function Starts and Stops the RCT Alarm B Timer to detect, while in Energy Storage State (EHS), to measure the time T_DCM_CCM.
	 * The time T_DC_CCM, defines when to switch from Discontinuous Charge Mode (DCM) to Continuous Charge Mode (CCM).
	 */

	if (mode == START_TIMER)
	{
		HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_B);	// Deactivate the RTC alarm B

		My_Set_RTC_Alarm_B
		(
				T_DCM_CCM_hours,
				T_DCM_CCM_minutes,
				T_DCM_CCM_seconds,
				T_DCM_CCM_subseconds
		);

#if MY_RTCB_Monitor
		My_RTCB_Monitor(Switch_CLOSED);
#endif
	}

	if (mode == STOP_TIMER)	// Stop EHS to EMS Timer
	{
		HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_B);	// Deactivate the RTC alarm B

#if MY_RTCB_Monitor
		My_RTCB_Monitor(Switch_OPEN);
#endif
	}
}

void My_EMS_to_EDS_Timer(My_Timer_td mode)
{
	/*
	 * This Function Starts the RCT Alarm B Timer to detect, while in Energy Missing State (EMS), to measure the time T_EMS_EDS.
	 * The time T_EMS_EDS, defines When to switch from Energy Missing State (EMS) to Energy Detection State (EDS), to perform an Energy presence check.
	 */

	if (mode == START_TIMER)
	{
		HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_B);	// Deactivate the RTC alarm B

		My_Set_RTC_Alarm_B
		(
				T_EMS_to_EDS_hours,
				T_EMS_to_EDS_minutes,
				T_EMS_to_EDS_seconds,
				T_EMS_to_EDS_subseconds
		);
#if MY_RTCB_Monitor
		My_RTCB_Monitor(Switch_CLOSED);
#endif
	}

	if (mode == STOP_TIMER)	// Stop EHS to EMS Timer
	{
		HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_B);	// Deactivate the RTC alarm B
#if MY_RTCB_Monitor
		My_RTCB_Monitor(Switch_OPEN);
#endif
	}
}

void My_EMS_to_SMS_Timer(My_Timer_td mode)
{
	/*
	 * This Function Starts the RTC Alarm A Timer to periodically switch into the Sensor Measurement State (SMS)
	 * while the system is in the Energy Missing State (EMS)
	 * */

	if (mode == START_TIMER)
	{
		HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);	// Deactivate the RTC alarm A
		My_Set_RTC_Alarm_A
		(
				T_EMS_SMS_hours,
				T_EMS_SMS_minutes,
				T_EMS_SMS_seconds,
				T_EMS_SMS_subseconds
		);
#if MY_RTCA_Monitor
		My_RTCA_Monitor(Switch_CLOSED);
#endif
	}

	if (mode == STOP_TIMER)	// Stop EHS to SMS Timer
	{
		HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);	// Deactivate the RTC alarm A
#if MY_RTCA_Monitor
		My_RTCA_Monitor(Switch_OPEN);
#endif
	}
}

/* Timer Section End */


/* Sensor Section Begin */

HAL_StatusTypeDef My_MX_I2C1_Init(I2C_HandleTypeDef* hi2c)
{
	HAL_StatusTypeDef ret = HAL_OK;

	hi2c->Instance = I2C1;
	hi2c->Init.Timing = I2C_CLOCK_SPEED;
	hi2c->Init.OwnAddress1 = 0;
	hi2c->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c->Init.OwnAddress2 = 0;
	hi2c->Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(hi2c) != HAL_OK)
	{
		ret = HAL_ERROR;
	}

	if (HAL_I2CEx_ConfigAnalogFilter(hi2c, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		ret = HAL_ERROR;
	}

	if (HAL_I2CEx_ConfigDigitalFilter(hi2c, 0) != HAL_OK)
	{
		ret = HAL_ERROR;
	}

	return ret;
}

void My_I2C1_MspInit(I2C_HandleTypeDef* i2cHandle)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
	/* USER CODE BEGIN I2C1_MspInit 0 */

	/* USER CODE END I2C1_MspInit 0 */

	/** Initializes the peripherals clocks
	 */
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
	PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

	__HAL_RCC_GPIOB_CLK_ENABLE();
	/**I2C1 GPIO Configuration
    PB7     ------> I2C1_SDA
    PB8     ------> I2C1_SCL
	 */
	GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;

	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* Peripheral clock enable */
	__HAL_RCC_I2C1_CLK_ENABLE();
	/* USER CODE BEGIN I2C1_MspInit 1 */

	/* USER CODE END I2C1_MspInit 1 */
}

void My_I2C1_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{
	/* USER CODE BEGIN I2C1_MspDeInit 0 */

	/* USER CODE END I2C1_MspDeInit 0 */
	/* Peripheral clock disable */
	__HAL_RCC_I2C1_CLK_DISABLE();

	/**I2C1 GPIO Configuration
    PB7     ------> I2C1_SDA
    PB8     ------> I2C1_SCL
	 */
	HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);

	HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8);

	/* USER CODE BEGIN I2C1_MspDeInit 1 */

	/* USER CODE END I2C1_MspDeInit 1 */
}


HAL_StatusTypeDef My_MX_I2C2_Init(I2C_HandleTypeDef* hi2c)
{
	HAL_StatusTypeDef ret = HAL_OK;

	hi2c->Instance = I2C2;
	hi2c->Init.Timing = I2C_CLOCK_SPEED;
	hi2c->Init.OwnAddress1 = 0;
	hi2c->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c->Init.OwnAddress2 = 0;
	hi2c->Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(hi2c) != HAL_OK)
	{
		ret = HAL_ERROR;
	}

	if (HAL_I2CEx_ConfigAnalogFilter(hi2c, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		ret = HAL_ERROR;
	}

	if (HAL_I2CEx_ConfigDigitalFilter(hi2c, 0) != HAL_OK)
	{
		ret = HAL_ERROR;
	}

	return ret;
}

void My_I2C2_MspInit(I2C_HandleTypeDef* i2cHandle)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
	/* USER CODE BEGIN I2C2_MspInit 0 */

	/* USER CODE END I2C2_MspInit 0 */

	/** Initializes the peripherals clocks
	 */
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
	PeriphClkInitStruct.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

	__HAL_RCC_GPIOA_CLK_ENABLE();
	/**I2C2 GPIO Configuration
    PA12     ------> I2C2_SCL
    PA15     ------> I2C2_SDA
	 */
	GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* Peripheral clock enable */
	__HAL_RCC_I2C2_CLK_ENABLE();
	/* USER CODE BEGIN I2C2_MspInit 1 */

	/* USER CODE END I2C2_MspInit 1 */
}

void My_I2C2_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{
	/* USER CODE BEGIN I2C2_MspDeInit 0 */

	/* USER CODE END I2C2_MspDeInit 0 */
	/* Peripheral clock disable */
	__HAL_RCC_I2C2_CLK_DISABLE();

	/**I2C2 GPIO Configuration
    PA12     ------> I2C2_SCL
    PA15     ------> I2C2_SDA
	 */
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_12);

	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_15);

	/* USER CODE BEGIN I2C2_MspDeInit 1 */

	/* USER CODE END I2C2_MspDeInit 1 */
}


void My_I2C_Init(My_I2C_td mode)
{
	switch (mode)
	{
	case MY_I2C1:
	{
		My_I2C1_MspInit(&hi2c1);
		My_MX_I2C1_Init(&hi2c1);
		break;
	}

	case MY_I2C2:
	{
		My_I2C2_MspInit(&hi2c2);
		My_MX_I2C2_Init(&hi2c2);
		break;
	}

	default:
	{
		break;
	}
	}
}

void My_I2C_Deinit(My_I2C_td mode)
{
	switch (mode)
	{
	case MY_I2C1:
	{
		My_I2C1_MspDeInit(&hi2c1);
		break;
	}

	case MY_I2C2:
	{
		My_I2C2_MspDeInit(&hi2c2);
		break;
	}

	default:
	{
		break;
	}
	}
}

#if MY_SHT40

void My_SHT40_Get_Data(void)
{
	my_SHT40_Temp = 0;																								// Reset my_SHT40_Temp
	my_SHT40_Humid = 0;																								// Reset my_SHT40_Humid

	uint8_t SHT40_Raw_Data[6];																						// Define raw data vector
	memset(SHT40_Raw_Data, 0, sizeof(SHT40_Raw_Data));																// Initialize raw data vector

	uint8_t SHT40_Measure_Cmd = SHT40_MEASURE_CMD_HP;																// Initialize SHT40 Address
	HAL_I2C_Master_Transmit(&hi2c2, SHT40AD1B_I2C_ADDRESS, &SHT40_Measure_Cmd, 1, HAL_MAX_DELAY); 					// Transmit Data through I2C2

	My_HAL_Delay(25);																								// Add delay to ensure the sensor is ready to provide data

	HAL_I2C_Master_Receive(&hi2c2, SHT40AD1B_I2C_ADDRESS, SHT40_Raw_Data, sizeof(SHT40_Raw_Data), HAL_MAX_DELAY);	// Receive Data through I2C2

	uint16_t SHT40_Raw_Temp = (SHT40_Raw_Data[0] << 8) | SHT40_Raw_Data[1];											// Temperature Data From SHT40
	uint16_t SHT40_Raw_Humid = (SHT40_Raw_Data[3] << 8) | SHT40_Raw_Data[4];										// Relative Humidity (RH) From SHT40

//	float my_SHT40_Temp_Float = -45 + 175 * ((float)SHT40_Raw_Temp / 65535);										// Temperature in Celsius
//	float my_SHT40_Humid_Float = -6 + 125 * ((float)SHT40_Raw_Humid / 65535);										// Humidity in %
	my_SHT40_Temp_Float = -45 + 175 * ((float)SHT40_Raw_Temp / 65535);										// Temperature in Celsius
	my_SHT40_Humid_Float = -6 + 125 * ((float)SHT40_Raw_Humid / 65535);										// Humidity in %

	if (my_SHT40_Humid_Float > 100.0f)																				// Clamp Humidity values to valid range (0-100%)
	{
		my_SHT40_Humid_Float = 100.0f;
	}

	if (my_SHT40_Humid_Float < 0.0f)																				// Clamp Humidity values to valid range (0-100%)
	{
		my_SHT40_Humid_Float = 0.0f;
	}

	my_SHT40_Temp =  (uint32_t)(10*my_SHT40_Temp_Float);															// Scale Temperature Value and convert to integer values
	my_SHT40_Humid = (uint32_t)(10*my_SHT40_Humid_Float);															// Scale Humidity Value and convert to integer values
}
#endif

#if MY_SoilSensor
void My_SoilSensor_Get_Data(void)
{
//	HAL_ADC_Start(&hadc);
//	HAL_ADC_PollForConversion(&hadc, 5000);
//	uint32_t adc_val = HAL_ADC_GetValue(&hadc);
//	HAL_ADC_Stop(&hadc);

	uint32_t adc_val = ADC_ReadChannels(ADC_CHANNEL_4);
//	Convert the ADC value to a percentage
	float adc = (float)adc_val;
	if (adc > AIR_VALUE) adc = AIR_VALUE;
	if (adc < WATER_VALUE) adc = WATER_VALUE;
	my_SoilSensor = 100.0f * (AIR_VALUE - adc) / (AIR_VALUE - WATER_VALUE);
}
#endif

#if MY_STTS22H

static int32_t My_STTS22H_Write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	HAL_StatusTypeDef ret;
	ret = HAL_I2C_Mem_Write(handle, STTS22H_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, HAL_MAX_DELAY);
	return ret;
}

static int32_t My_STTS22H_Read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	HAL_StatusTypeDef ret;
	ret  = HAL_I2C_Mem_Read(handle, STTS22H_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, HAL_MAX_DELAY);
	return ret;
}

void My_STTS22H_Get_Data(void)
{
	stmdev_ctx_t stts22h_ctx;																		// Define and initialize the sensor context
	stts22h_ctx.write_reg = My_STTS22H_Write;														// Write function
	stts22h_ctx.read_reg = My_STTS22H_Read;															// Read function
	stts22h_ctx.handle = &hi2c2;																	// Communication bus handle

	my_STTS22H_Sensor_Flag = 0;																	// Reset Sensor Presence Flag
	my_STTS22H_Temp = 0;																			// Reset my_STTS22H_Temp

	uint8_t stts22h_id = 0;																			// Initialize stts22h_whoami
	int16_t my_STTS22H_Raw_Data = 0;																// Initialize raw data

	uint8_t my_STTS22H_Ctrl_Reg = STTS22H_CTRL_REG_VALUE;
	stts22h_write_reg(&stts22h_ctx, STTS22H_CTRL, &my_STTS22H_Ctrl_Reg, 1);		 					// Configure the sensor for measurement
	My_HAL_Delay(25);																				// Add a Delay

	stts22h_dev_id_get(&stts22h_ctx, &stts22h_id);													// Get Device ID
	if (stts22h_id == STTS22H_ID)																	// Check Sensor Presence
	{
		my_STTS22H_Sensor_Flag = 1;																// Assert Sensor Presence Flag

		stts22h_temperature_raw_get(&stts22h_ctx, &my_STTS22H_Raw_Data);							// Read raw temperature data from the sensor

		float my_STTS22H_Temperature_Celsius = stts22h_from_lsb_to_celsius(my_STTS22H_Raw_Data);	// Convert raw data to Celsius

		my_STTS22H_Temp = (uint32_t)(my_STTS22H_Temperature_Celsius);								// Convert to integer values
	}
}
#endif

#if MY_STHS34PF80
static int32_t My_STHS34PF80_Write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	HAL_StatusTypeDef ret;
	ret = HAL_I2C_Mem_Write(handle, STHS34PF80_I2C_ADD, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, HAL_MAX_DELAY);
	return ret;
}

static int32_t My_STHS34PF80_Read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	HAL_StatusTypeDef ret;
	ret = HAL_I2C_Mem_Read(handle, STHS34PF80_I2C_ADD, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, HAL_MAX_DELAY);
	return ret;
}

void My_STHS34PF80_Presence_Motion_Cfg(stmdev_ctx_t *ctx)
{
	// Read the Low-Pass Filters
	sths34pf80_lpf_bandwidth_t my_lpf_p_m, my_lpf_m, my_lpf_p, my_lpf_a_t;

	sths34pf80_lpf_m_bandwidth_get(ctx, &my_lpf_m);
	sths34pf80_lpf_p_bandwidth_get(ctx, &my_lpf_p);
	sths34pf80_lpf_p_m_bandwidth_get(ctx, &my_lpf_p_m);
	sths34pf80_lpf_a_t_bandwidth_get(ctx, &my_lpf_a_t);

	// Set Thresholds
	sths34pf80_presence_threshold_set(ctx, 200);     	  // Set Presence threshold
	sths34pf80_presence_hysteresis_set(ctx, 20);  	      // Set Presence hysteresis
	sths34pf80_motion_threshold_set(ctx, 300);            // Set Motion threshold
	sths34pf80_motion_hysteresis_set(ctx, 30);            // Set Motion hysteresis
	sths34pf80_tambient_shock_threshold_set(ctx, 50);     // Set Ambient Temperature Shock threshold
	sths34pf80_tambient_shock_hysteresis_set(ctx, 10);    // Set Ambient Temperature Shock hysteresis
	sths34pf80_reset_algo(ctx);                           // Reset algorithms

	// Set and get the presence absolute value
	uint8_t abs_presence_value_set = 1;
	uint8_t abs_presence_value_get = 0;
	sths34pf80_presence_abs_value_set(ctx, abs_presence_value_set);
	sths34pf80_presence_abs_value_get(ctx, &abs_presence_value_get);

	// Variables to store thresholds and hysteresis values
	uint16_t presence_threshold, motion_threshold, temp_shock_threshold;
	uint8_t presence_hysteresis, motion_hysteresis, temp_shock_hysteresis;

	// Get the configured thresholds and hysteresis values
	sths34pf80_presence_threshold_get(ctx, &presence_threshold);
	sths34pf80_presence_hysteresis_get(ctx, &presence_hysteresis);

	sths34pf80_motion_threshold_get(ctx, &motion_threshold);
	sths34pf80_motion_hysteresis_get(ctx, &motion_hysteresis);

	sths34pf80_tambient_shock_threshold_get(ctx, &temp_shock_threshold);
	sths34pf80_tambient_shock_hysteresis_get(ctx, &temp_shock_hysteresis);
}

void My_STHS34PF80_Get_Data(void)
{
	stmdev_ctx_t sths34pf80_ctx; 					// Define and initialize the sensor context
	sths34pf80_ctx.write_reg = My_STHS34PF80_Write;	// Write function
	sths34pf80_ctx.read_reg = My_STHS34PF80_Read;   // Read function
	sths34pf80_ctx.handle = &hi2c2;					// Communication bus handle

	my_STHS34PF80_Sensor_Flag = 0;		// Reset Sensor Presence Flag
	my_STHS34PF80_Obj_Temp = 0;         // Reset Object Temperature Value
	my_STHS34PF80_Amb_Temp = 0;         // Reset Ambient Temperature Value
	my_STHS34PF80_Motion = 0;         	// Reset Motion Flag
	my_STHS34PF80_Presence = 0;      	// Reset Presence Flag
	my_STHS34PF80_Tamb_Shock = 0;		// Reset Ambient Temperature Shock Flag
	my_STHS34PF80_Presence_Data = 0;	// Reset Presence Data
	my_STHS34PF80_Motion_Data = 0;		// Reset Motion Data

	uint8_t my_sths34pf80_id = 0;												// Initialize my_sths34pf80_id
	uint8_t my_STHS34PF80_Raw_Data[6];  										// Define Data Buffer
	memset(my_STHS34PF80_Raw_Data, 0, sizeof(my_STHS34PF80_Raw_Data));			// Initialize Data Buffer

	sths34pf80_avg_tobject_num_set(&sths34pf80_ctx, STHS34PF80_AVG_TMOS_32);	// Set the number of averages for object temperature
	My_HAL_Delay(25);															// Add a Delay

	sths34pf80_avg_tambient_num_set(&sths34pf80_ctx, STHS34PF80_AVG_T_8);		// Set the number of averages for ambient temperature
	My_HAL_Delay(25);															// Add a Delay

	sths34pf80_tmos_odr_set(&sths34pf80_ctx, STHS34PF80_TMOS_ODR_AT_30Hz);		// Set the object temperature sensor output data rate (ODR) to 30Hz
	My_HAL_Delay(25);															// Add a Delay

	sths34pf80_block_data_update_set(&sths34pf80_ctx, 1);						// Enable Block Data Update (BDU)
	My_HAL_Delay(25);															// Add a Delay

	sths34pf80_tmos_one_shot_set(&sths34pf80_ctx, 0);							// Set the temperature sensor to continuous mode (0) instead of one-shot mode (1)
	My_HAL_Delay(25);															// Add a Delay

#if MY_STHS34PF80_CONFIG
	My_STHS34PF80_Presence_Motion_Cfg(&sths34pf80_ctx);  						// Configure Presence and Motion Thresholds
#endif

	sths34pf80_device_id_get(&sths34pf80_ctx, &my_sths34pf80_id);						// Get the device ID

	if (my_sths34pf80_id == STHS34PF80_ID)												// Check if Sensor is Present
	{
		my_STHS34PF80_Sensor_Flag = 1;												// Assert STHS34PF80 Sensor Presence

		uint16_t my_STHS34PF80_Sensitivity = 0;
		sths34pf80_tmos_sensitivity_get(&sths34pf80_ctx, &my_STHS34PF80_Sensitivity); 	// Get STHS34PF80 Sensor Sensitivity

		int16_t raw_Obj_Temp_Data = 0;
		sths34pf80_tobject_raw_get(&sths34pf80_ctx, &raw_Obj_Temp_Data);				// Get Raw Data for the Object Temperature

		float my_STHS34PF80_Obj_Temp_Float = 0.0f;

		if (my_STHS34PF80_Sensitivity != 0)
		{
			my_STHS34PF80_Obj_Temp_Float = (float)raw_Obj_Temp_Data / (float)my_STHS34PF80_Sensitivity;
		}

		if (my_STHS34PF80_Obj_Temp_Float >= 0)
		{
			my_STHS34PF80_Obj_Temp = (uint32_t)(my_STHS34PF80_Obj_Temp_Float * 100);  	// Scale by 100 for Precision
		}

		int16_t raw_Amb_Temp_Data = 0;
		sths34pf80_tambient_raw_get(&sths34pf80_ctx, &raw_Amb_Temp_Data);				// Get Raw Ambient Temperature Data From STHS34PF80
		my_STHS34PF80_Amb_Temp = (uint32_t)raw_Amb_Temp_Data;							// Temperature in Celsius scaled by 100

		int16_t raw_Presence_Data = 0;
		sths34pf80_tpresence_raw_get(&sths34pf80_ctx, &raw_Presence_Data);

		if (raw_Presence_Data >= 0)
		{
			my_STHS34PF80_Presence_Data = (uint16_t)(raw_Presence_Data);
		}

		int16_t raw_Motion_Data = 0;
		sths34pf80_tmotion_raw_get(&sths34pf80_ctx, &raw_Motion_Data);

		if (raw_Motion_Data >= 0)
		{
			my_STHS34PF80_Motion_Data = (uint16_t)(raw_Motion_Data);
		}

		int16_t raw_Tamb_Shock = 0;
		sths34pf80_tamb_shock_raw_get(&sths34pf80_ctx, & raw_Tamb_Shock);

		sths34pf80_tmos_func_status_t func_status;										// Get Presence and Motion an Tamb_Shock Data
		sths34pf80_tmos_func_status_get(&sths34pf80_ctx, &func_status);

		my_STHS34PF80_Presence = func_status.pres_flag;

		my_STHS34PF80_Motion = func_status.mot_flag;

		my_STHS34PF80_Tamb_Shock = func_status.tamb_shock_flag;
	}
}

#endif

#if MY_LIS2DU12

int32_t My_LIS2DU12_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	HAL_StatusTypeDef ret;
	ret = HAL_I2C_Mem_Write(handle, LIS2DU12_I2C_ADD_H, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, HAL_MAX_DELAY);
	return ret;
}

int32_t My_LIS2DU12_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	HAL_StatusTypeDef ret;
	ret = HAL_I2C_Mem_Read(handle, LIS2DU12_I2C_ADD_H, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, HAL_MAX_DELAY);
	return ret;
}

void My_LIS2DU12_Get_Data(void)
{
	stmdev_ctx_t lis2du12_ctx;										// Define and initialize the sensor context
	lis2du12_ctx.write_reg = My_LIS2DU12_write;						// Write function
	lis2du12_ctx.read_reg = My_LIS2DU12_read;						// Read function
	lis2du12_ctx.handle = &hi2c1;									// Communication bus handle

	lis2du12_id_t lis2du12_id;  									// Define lis2du12 id
	lis2du12_id.whoami = 0;  										// Initialize lis2du12 to 0
	uint8_t my_LIS2DU12_raw_data[6];								// Define Data Buffer
	memset(my_LIS2DU12_raw_data, 0, sizeof(my_LIS2DU12_raw_data));	// Initialize Data Buffer

	my_LIS2DU12_Sensor_Flag = 0;									// Reset my_lis2du12_presence
	my_LIS2DU12_acc_x = 0;											// Reset lis2du12_acc_x
	my_LIS2DU12_acc_y = 0;											// Reset lis2du12_acc_y
	my_LIS2DU12_acc_z = 0;											// Reset lis2du12_acc_z

	lis2du12_id_get(&lis2du12_ctx, &lis2du12_id);					// Get Device ID

	if (lis2du12_id.whoami == LIS2DU12_ID)							// Check Presence of LIS2DU12
	{
		my_LIS2DU12_Sensor_Flag = 1;

		uint8_t ctrl1_value = LIS2DU12_CTRL_REG1_VALUE;
		uint8_t ctrl2_value = LIS2DU12_CTRL_REG2_VALUE;
		uint8_t ctrl3_value = LIS2DU12_CTRL_REG3_VALUE;
		uint8_t ctrl4_value = LIS2DU12_CTRL_REG4_VALUE;
		uint8_t ctrl5_value = LIS2DU12_CTRL_REG5_VALUE;
		uint8_t fifo_ctrl_value = LIS2DU12_FIFO_CTRL_VALUE;

		// Write the values to the control registers
		My_LIS2DU12_write(&hi2c1, LIS2DU12_CTRL1, &ctrl1_value, 1);
		My_HAL_Delay(20);	// Add delay to ensure the sensor is ready to provide data

		My_LIS2DU12_write(&hi2c1, LIS2DU12_CTRL2, &ctrl2_value, 1);
		My_HAL_Delay(20);	// Add delay to ensure the sensor is ready to provide data

		My_LIS2DU12_write(&hi2c1, LIS2DU12_CTRL3, &ctrl3_value, 1);
		My_HAL_Delay(20);	// Add delay to ensure the sensor is ready to provide data

		My_LIS2DU12_write(&hi2c1, LIS2DU12_CTRL4, &ctrl4_value, 1);
		My_HAL_Delay(20);	// Add delay to ensure the sensor is ready to provide data

		My_LIS2DU12_write(&hi2c1, LIS2DU12_CTRL5, &ctrl5_value, 1);
		My_HAL_Delay(20);	// Add delay to ensure the sensor is ready to provide data

		My_LIS2DU12_write(&hi2c1, LIS2DU12_FIFO_CTRL, &fifo_ctrl_value, 1);
		My_HAL_Delay(20);	// Add delay to ensure the sensor is ready to provide data

		My_LIS2DU12_read(&hi2c1, LIS2DU12_OUTX_L | 0x80, my_LIS2DU12_raw_data, sizeof(my_LIS2DU12_raw_data)); // Read Data From LIS2DU12

		my_LIS2DU12_acc_x = (uint16_t)(my_LIS2DU12_raw_data[0] | (my_LIS2DU12_raw_data[1] << 8));	// Acceleration Data on x Axis
		my_LIS2DU12_acc_y = (uint16_t)(my_LIS2DU12_raw_data[2] | (my_LIS2DU12_raw_data[3] << 8));	// Acceleration Data on y Axis
		my_LIS2DU12_acc_z = (uint16_t)(my_LIS2DU12_raw_data[4] | (my_LIS2DU12_raw_data[5] << 8));	// Acceleration Data on z Axis
	}
}

#endif

/* Sensor Section End */

void My_Set_All_GPIO_To_Analog_Mode(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Enable clocks for all available GPIO ports
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();

	// Configure all pins of the GPIO ports as analog to reduce power consumption
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Pin = GPIO_PIN_All;

	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

	// Disable clocks to save power
	__HAL_RCC_GPIOA_CLK_DISABLE();
	__HAL_RCC_GPIOB_CLK_DISABLE();
	__HAL_RCC_GPIOC_CLK_DISABLE();
	__HAL_RCC_GPIOH_CLK_DISABLE();
}

void My_Enter_Stop2_Mode_WFI(uint32_t PWR_PVDLEVEL, uint32_t PWR_PVD_MODE_IT)	// Enter in STOP2 MODE and Wait for Interrupts (WFI)
{

	sConfigPVD.PVDLevel = PWR_PVDLEVEL; 							// Set PVD Threshold
	sConfigPVD.Mode = PWR_PVD_MODE_IT;								// Config PVD to provide Interrupt

	HAL_PWR_ConfigPVD(&sConfigPVD);									// Configure PVD

	if (my_MX_GPIO_Init_done == 1)
	{
		BSP_RADIO_DeInit();											// Deinit the External Radio Amplifier
	}

	if (my_SystemClock_Config_done == 1)							// Check if the Clock System has been configured
	{
		HAL_SuspendTick();											// Suspend Tick increment of System Clock
		__HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_MSI); 	// Ensure that MSI is wake-up system clock
	}

	if((LL_PWR_IsActiveFlag_C1SB() == 0) || (LL_PWR_IsActiveFlag_C2SB() == 0))
	{
		LL_PWR_ClearFlag_C1STOP_C1STB();			  				// Clear standby and stop flags for CPU1
		LL_PWR_ClearFlag_C2STOP_C2STB();							// Clear standby and stop flags for CPU2
		LL_C2_PWR_SetPowerMode(LL_PWR_MODE_SHUTDOWN); 				// Set the lowest low-power mode for CPU2: shutdown mode
		LL_PWR_SetPowerMode(LL_PWR_MODE_STOP2);						// Set Low-Power mode for CPU1
	}

	My_Interrupts_Manager(NVIC_DISABLE);							// Disable and Clear All Pending Interrupts
	HAL_NVIC_ClearPendingIRQ(PVD_PVM_IRQn);							// Clear Pending Bit for PVD
	HAL_NVIC_ClearPendingIRQ(RTC_Alarm_IRQn);						// Clear Pending Bit for RTC Alarms (A and B) Interrupt
	HAL_NVIC_ClearPendingIRQ(COMP_IRQn);							// Clear Pending Bit for Comparator

	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_PVDO);							// CLear Power Voltage Detector output Flag
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);								// CLear Flag
	HAL_NVIC_EnableIRQ(PVD_PVM_IRQn);								// Enable Interrupt for PVD
	HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);								// Enable Interrupt for Interrupt for RTC Alarms (A and B) Interrupt
	HAL_NVIC_EnableIRQ(LPTIM1_IRQn);								// Enable Interrupt for LPTIM1

	if (__HAL_PWR_GET_FLAG(PWR_CR1_LPR) == 1)						// Check if PWR_CR1_LPR is set (must be 0 in Stop2 Mode)
	{
		CLEAR_BIT(PWR->CR1, PWR_CR1_LPR); 							// LPR bit must be cleared to enter stop 2 mode.
	}

	HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);					// Enter STOP2 mode and WFI

	if (my_SubghzApp_Init_done == 0)
	{
		My_Exit_Stop2_Mode_WFI();									// Exit From Stop2 Mode
	}
}

void My_Exit_Stop2_Mode_WFI(void)	// Exit Stop2 Mode After Interrupt
{
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);		// Undervolting
	My_Interrupts_Manager(NVIC_ENABLE);									// Re-Enable Interrupts
	if (my_SystemClock_Config_done == 1)
	{
		HAL_ResumeTick();												// Resume Tick Increment
	}
}

void My_Set_PVD(uint32_t PWR_PVDLEVEL, uint32_t PWR_PVD_MODE_IT)	// Sets PVD Threshold and Interrupt Modes
{
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_PVDO);							// CLear Power Voltage Detector output Flag
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);								// CLear Flag
	sConfigPVD.PVDLevel = PWR_PVDLEVEL; 							// Set PVD Threshold
	sConfigPVD.Mode = PWR_PVD_MODE_IT;								// Set PVD to provide Interrupt
	HAL_PWR_ConfigPVD(&sConfigPVD);									// Configure PVD
}

void My_Set_PVD_WFI(uint32_t PWR_PVDLEVEL, uint32_t PWR_PVD_MODE_IT) // Sets PVD Threshold and Interrupt Modes and WFI
{
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_PVDO);							// CLear Power Voltage Detector output Flag
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);								// CLear Flag
	sConfigPVD.PVDLevel = PWR_PVDLEVEL; 							// Set PVD Threshold
	sConfigPVD.Mode = PWR_PVD_MODE_IT;								// Set PVD to provide Interrupt
	HAL_PWR_ConfigPVD(&sConfigPVD);									// Configure PVD

	My_Interrupts_Manager(NVIC_DISABLE);							// Disable and Clear All Pending Interrupts

	HAL_NVIC_EnableIRQ(PVD_PVM_IRQn);								// Enable Interrupt for PVD

	__WFI(); 														// Wait for the Interrupt
}

void My_Interrupts_Manager(My_Interrupts_Manager_td My_Interrupts_Status)	// Enable or Disable IRQ
{
	switch (My_Interrupts_Status)
	{
	case NVIC_DISABLE:
	{

		HAL_NVIC_DisableIRQ(PVD_PVM_IRQn);									// Disable Interrupt for PVD
		HAL_NVIC_DisableIRQ(RTC_WKUP_IRQn);									// Disable Interrupt for RTC_WKUP
		HAL_NVIC_DisableIRQ(TAMP_STAMP_LSECSS_SSRU_IRQn);					// Disable Interrupt for RTC Tamper, RTC TimeStamp, LSECSS and RTC SSRU Interrupts
		HAL_NVIC_DisableIRQ(DMA1_Channel5_IRQn);							// Disable Interrupt for DMA1 Channel 5 Interrupt
		HAL_NVIC_DisableIRQ(USART2_IRQn);									// Disable Interrupt for USART2
		HAL_NVIC_DisableIRQ(RTC_Alarm_IRQn);								// Disable Interrupt for RTC Alarms (A and B) Interrupt
		HAL_NVIC_DisableIRQ(SUBGHZ_Radio_IRQn);								// Disable Interrupt for SUBGHZ Radio Interrupt
		HAL_NVIC_DisableIRQ(LPTIM1_IRQn);									// Disable Interrupt for LPTIM1

		HAL_NVIC_ClearPendingIRQ(PVD_PVM_IRQn);								// Clear Pending Bit for PVD
		HAL_NVIC_ClearPendingIRQ(RTC_WKUP_IRQn);							// Clear Pending Bit for RTC_WKUP
		HAL_NVIC_ClearPendingIRQ(TAMP_STAMP_LSECSS_SSRU_IRQn);				// Clear Pending Bit for RTC Tamper, RTC TimeStamp, LSECSS and RTC SSRU Interrupts
		HAL_NVIC_ClearPendingIRQ(DMA1_Channel5_IRQn);						// Clear Pending Bit for DMA1 Channel 5 Interrupt
		HAL_NVIC_ClearPendingIRQ(USART2_IRQn);								// Clear Pending Bit for USART2
		HAL_NVIC_ClearPendingIRQ(RTC_Alarm_IRQn);							// Clear Pending Bit for RTC Alarms (A and B) Interrupt
		HAL_NVIC_ClearPendingIRQ(SUBGHZ_Radio_IRQn);						// Clear Pending Bit for SUBGHZ Radio Interrupt
		HAL_NVIC_ClearPendingIRQ(LPTIM1_IRQn);								// Clear Pending Bit for LPTIM1 Interrupt

		break;
	}

	case NVIC_ENABLE:
	{
		HAL_NVIC_DisableIRQ(PVD_PVM_IRQn);									// Disable Interrupt for PVD
		HAL_NVIC_DisableIRQ(RTC_WKUP_IRQn);									// Disable Interrupt for RTC_WKUP
		HAL_NVIC_DisableIRQ(TAMP_STAMP_LSECSS_SSRU_IRQn);					// Disable Interrupt for RTC Tamper, RTC TimeStamp, LSECSS and RTC SSRU Interrupts
		HAL_NVIC_DisableIRQ(DMA1_Channel5_IRQn);							// Disable Interrupt for DMA1 Channel 5 Interrupt
		HAL_NVIC_DisableIRQ(USART2_IRQn);									// Disable Interrupt for USART2
		HAL_NVIC_DisableIRQ(RTC_Alarm_IRQn);								// Disable Interrupt for RTC Alarms (A and B) Interrupt
		HAL_NVIC_DisableIRQ(SUBGHZ_Radio_IRQn);								// Disable Interrupt for SUBGHZ Radio Interrupt
		HAL_NVIC_DisableIRQ(LPTIM1_IRQn);									// Disable Interrupt for LPTIM1

		HAL_NVIC_ClearPendingIRQ(PVD_PVM_IRQn);								// Clear Pending Bit for PVD
		HAL_NVIC_ClearPendingIRQ(RTC_WKUP_IRQn);							// Clear Pending Bit for RTC_WKUP
		HAL_NVIC_ClearPendingIRQ(TAMP_STAMP_LSECSS_SSRU_IRQn);				// Clear Pending Bit for RTC Tamper, RTC TimeStamp, LSECSS and RTC SSRU Interrupts
		HAL_NVIC_ClearPendingIRQ(DMA1_Channel5_IRQn);						// Clear Pending Bit for DMA1 Channel 5 Interrupt
		HAL_NVIC_ClearPendingIRQ(USART2_IRQn);								// Clear Pending Bit for USART2
		HAL_NVIC_ClearPendingIRQ(RTC_Alarm_IRQn);							// Clear Pending Bit for RTC Alarms (A and B) Interrupt
		HAL_NVIC_ClearPendingIRQ(SUBGHZ_Radio_IRQn);						// Clear Pending Bit for SUBGHZ Radio Interrupt
		HAL_NVIC_ClearPendingIRQ(LPTIM1_IRQn);								// Clear Pending Bit for LPTIM1 Interrupt

		HAL_NVIC_SetPriority(PVD_PVM_IRQn, 0, 0);							// Set Interrupt Priority
		HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 1, 0);							// Set Interrupt Priority
		HAL_NVIC_SetPriority(TAMP_STAMP_LSECSS_SSRU_IRQn, 1, 0);			// Set Interrupt Priority
		HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 1, 0);						// Set Interrupt Priority
		HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);							// Set Interrupt Priority
		HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 1, 0);							// Set Interrupt Priority
		HAL_NVIC_SetPriority(SUBGHZ_Radio_IRQn, 1, 0);						// Set Interrupt Priority
		HAL_NVIC_SetPriority(LPTIM1_IRQn, 1, 0);							// Set Interrupt Priority

		HAL_NVIC_EnableIRQ(PVD_PVM_IRQn);									// Enable Interrupt for PVD
		HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);									// Enable Interrupt for RTC_WKUP
		HAL_NVIC_EnableIRQ(TAMP_STAMP_LSECSS_SSRU_IRQn);					// Enable Interrupt for RTC Tamper, RTC TimeStamp, LSECSS and RTC SSRU Interrupts
		HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);								// Enable Interrupt for SUBGHZ Radio Interrupt
		HAL_NVIC_EnableIRQ(USART2_IRQn);									// Enable Interrupt for USART2
		HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);									// Enable Interrupt for Interrupt for RTC Alarms (A and B) Interrupt
		HAL_NVIC_EnableIRQ(SUBGHZ_Radio_IRQn);								// Enable Interrupt for SUBGHZ Radio Interrupt
		HAL_NVIC_EnableIRQ(LPTIM1_IRQn);									// Enable Interrupt for LPTIM1

		break;
	}

	case NVIC_CLEAR:
	{
		HAL_NVIC_DisableIRQ(PVD_PVM_IRQn);									// Disable Interrupt for PVD
		HAL_NVIC_DisableIRQ(RTC_WKUP_IRQn);									// Disable Interrupt for RTC_WKUP
		HAL_NVIC_DisableIRQ(TAMP_STAMP_LSECSS_SSRU_IRQn);					// Disable Interrupt for RTC Tamper, RTC TimeStamp, LSECSS and RTC SSRU Interrupts
		HAL_NVIC_DisableIRQ(DMA1_Channel5_IRQn);							// Disable Interrupt for DMA1 Channel 5 Interrupt
		HAL_NVIC_DisableIRQ(USART2_IRQn);									// Disable Interrupt for USART2
		HAL_NVIC_DisableIRQ(RTC_Alarm_IRQn);								// Disable Interrupt for RTC Alarms (A and B) Interrupt
		HAL_NVIC_DisableIRQ(SUBGHZ_Radio_IRQn);								// Disable Interrupt for SUBGHZ Radio Interrupt
		HAL_NVIC_DisableIRQ(LPTIM1_IRQn);									// Disable Interrupt for LPTIM1

		HAL_NVIC_ClearPendingIRQ(PVD_PVM_IRQn);								// Clear Pending Bit for PVD
		HAL_NVIC_ClearPendingIRQ(RTC_WKUP_IRQn);							// Clear Pending Bit for RTC_WKUP
		HAL_NVIC_ClearPendingIRQ(TAMP_STAMP_LSECSS_SSRU_IRQn);				// Clear Pending Bit for RTC Tamper, RTC TimeStamp, LSECSS and RTC SSRU Interrupts
		HAL_NVIC_ClearPendingIRQ(DMA1_Channel5_IRQn);						// Clear Pending Bit for DMA1 Channel 5 Interrupt
		HAL_NVIC_ClearPendingIRQ(USART2_IRQn);								// Clear Pending Bit for USART2
		HAL_NVIC_ClearPendingIRQ(RTC_Alarm_IRQn);							// Clear Pending Bit for RTC Alarms (A and B) Interrupt
		HAL_NVIC_ClearPendingIRQ(SUBGHZ_Radio_IRQn);						// Clear Pending Bit for SUBGHZ Radio Interrupt
		HAL_NVIC_ClearPendingIRQ(LPTIM1_IRQn);								// Clear Pending Bit for LPTIM1 Interrupt

		HAL_NVIC_EnableIRQ(PVD_PVM_IRQn);									// Enable Interrupt for PVD
		HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);									// Enable Interrupt for RTC_WKUP
		HAL_NVIC_EnableIRQ(TAMP_STAMP_LSECSS_SSRU_IRQn);					// Enable Interrupt for RTC Tamper, RTC TimeStamp, LSECSS and RTC SSRU Interrupts
		HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);								// Enable Interrupt for SUBGHZ Radio Interrupt
		HAL_NVIC_EnableIRQ(USART2_IRQn);									// Enable Interrupt for USART2
		HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);									// Enable Interrupt for Interrupt for RTC Alarms (A and B) Interrupt
		HAL_NVIC_EnableIRQ(SUBGHZ_Radio_IRQn);								// Enable Interrupt for SUBGHZ Radio Interrupt
		HAL_NVIC_EnableIRQ(LPTIM1_IRQn);									// Enable Interrupt for LPTIM1

		break;
	}

	default:
	{
		break;
	}

	}
}

void My_Set_RTC_Alarm_A(uint8_t my_hours, uint8_t my_minutes, uint8_t my_seconds, uint32_t my_subseconds)
{

	/* This function Sets the RTC ALARM A Timer */

	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

	sAlarm.AlarmTime.Hours = sTime.Hours + my_hours;
	sAlarm.AlarmTime.Minutes = my_minutes + sTime.Minutes;
	sAlarm.AlarmTime.Seconds = my_seconds + sTime.Seconds;
	sAlarm.AlarmTime.SubSeconds = my_subseconds + sTime.SubSeconds;
	sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
	sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
	sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
	sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
	sAlarm.AlarmDateWeekDay = 0x1;
	sAlarm.Alarm = RTC_ALARM_A;

	HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN);			// Set RTC Alarm Time with Interrupt
}

void My_Set_RTC_Alarm_B(uint8_t my_hours, uint8_t my_minutes, uint8_t my_seconds, uint32_t my_subseconds)
{
	/* Set the RTC ALARM B Timer */

	sTime.Hours = 0x00; 														// Config Time Parameter RTC Struct Init
	sTime.Minutes = 0x00;
	sTime.Seconds = 0x00;
	sTime.SubSeconds = 0x00;
	sTime.SecondFraction = 0x00;
	sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sTime.StoreOperation = RTC_STOREOPERATION_RESET;

	sAlarm.AlarmTime.Hours = my_hours;											// Config Time Parameter of RTC_Alarm Struct
	sAlarm.AlarmTime.Minutes = my_minutes;
	sAlarm.AlarmTime.Seconds = my_seconds;
	sAlarm.AlarmTime.SubSeconds = my_subseconds;
	sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
	sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
	sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
	sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
	sAlarm.AlarmDateWeekDay = 0x1;
	sAlarm.Alarm = RTC_ALARM_B;

	HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);								// Set RTC Time (this Command needed for Set Alarm)
	HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN);						// Set RTC Alarm Time with Interrupt
}

void My_SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_7;				// Set Clock Frequency Below 16 MHz to allow Undervolting
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
			|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
			|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;


	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

void My_SystemClock_Update(uint32_t CLOCK_RANGE)
{
	// This Function updates the Clock Frequency in RunTime

	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.MSIClockRange = CLOCK_RANGE;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
			|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
			|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;


	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

void My_GPIO_Out_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	//	/*Configure GPIO pin as Output: PA13 for Debug Purposes */
	GPIO_InitStruct.Pin = IMPEDANCE_Meas_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOA, IMPEDANCE_Meas_Pin, GPIO_PIN_RESET);
}

void My_Enable_HSI(void) //Enable the HSI at 1MHz
{
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);
}

void My_Disable_HSI(void) //Disable the HSI at 1MHz
{
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);
}

void My_Set_GPIO_ANALOG(void)
{
	SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_GPIOAEN);
	SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_GPIOBEN);
	SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_GPIOCEN);
	GPIOA -> MODER = 0xFFFFFFFF;
	GPIOB -> MODER = 0xFFFFFFFF;
	GPIOC -> MODER = 0xFFFFFFFF;
	CLEAR_BIT(RCC->AHB2ENR, RCC_AHB2ENR_GPIOAEN);
	CLEAR_BIT(RCC->AHB2ENR, RCC_AHB2ENR_GPIOBEN);
	CLEAR_BIT(RCC->AHB2ENR, RCC_AHB2ENR_GPIOCEN);
}

void My_Trise_Meas(void)
{
	/* This Function measures the charging time (Trise) during the Energy Harvesting State (EHS) from the last SMS State to the following RTS State */

	my_Trise_time_stop = HAL_LPTIM_ReadCounter(&hlptim1);// Stop Measuring Trise

	if (my_Trise_time_stop > my_Trise_time_start)
	{
		my_Trise_time = my_Trise_time_stop - my_Trise_time_start;
	}
	else
	{
		if (my_Cmode == EOC)
		{
			my_Trise_time = my_Trise_time_stop + (T_ESW + T_SMW) - my_Trise_time_start;
		}
		else
		{
			my_Trise_time = my_Trise_time_stop + (T_SMW) - my_Trise_time_start;
		}
	}
}

void My_PVD_Delay(void)
{
	/* This Function creates a Delay to allow the PVD to settle */

	HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A); 	// Deactivate the RTC Alarm A

	My_Set_RTC_Alarm_A								// Sets the RTC Alarm A Timer to create a Delay of time T_Delay_PVD
	(
			T_Delay_PVD_hours,
			T_Delay_PVD_minutes,
			T_Delay_PVD_seconds,
			T_Delay_PVD_subseconds
	);
	__WFI(); 										// Wait for the Interrupt

	HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);   	// Deactivate the RTC Alarm A
}

void My_PS_Detection(void)
{
	My_Set_PVD(VPS_CHECK, PWR_PVD_MODE_NORMAL);  	  					// Configures PVD at the Highest Voltage and Normal Mode

	if (__HAL_PWR_GET_FLAG(PWR_FLAG_PVDO) == 0)							// If VDD is > 2.9 Volt and and Internal Reference is Ready
	{
		my_Current_State = PSS;											// System is Supplied by a Power Supply with voltage > 3.0 V
	}
	else
	{
		my_Current_State = EHS;											// Set Current State as Energy Harvesting State (EHS)
		my_Next_State = RTS;											// Set Next State as Radio Transmission State (RTS)
		My_Enter_Stop2_Mode_WFI(VRTS, PWR_PVD_MODE_IT_RISING);			// Enter STOP2 Mode and WFI
//		My_Enter_Stop2_Mode_WFI(V2P5, PWR_PVD_MODE_IT_RISING);
	}
}


#if STEVAL_HARVEST1
void My_VDD_to_ES_Switch(My_Switch_td mode)	// Opens and Closes the switches (High side of a GPIO) between Vdd and ES net
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();

	if (mode == Switch_OPEN)
	{
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;

		/* Configure GPIO Pins: PC13 */
		GPIO_InitStruct.Pin =  ES_Pin4;
		HAL_GPIO_Init(ES_PortC, &GPIO_InitStruct);

		/* Configure GPIO Pins: PB3, PB4, PB14 */
		GPIO_InitStruct.Pin = ES_Pin1 | ES_Pin2 | ES_Pin3;
		HAL_GPIO_Init(ES_PortB, &GPIO_InitStruct);
	}

	if (mode == Switch_CLOSED)
	{
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;

		/* Configure GPIO Pins: PC13 */
		GPIO_InitStruct.Pin =  ES_Pin4;
		HAL_GPIO_Init(ES_PortC, &GPIO_InitStruct);
		HAL_GPIO_WritePin(ES_PortC, ES_Pin4, GPIO_PIN_SET); // Close the High Side of the GPIO to connect ES_Pin4 to Vdd

		/* Configure GPIO Pins: PB3, PB4, PB14 */
		GPIO_InitStruct.Pin = ES_Pin1 | ES_Pin2 | ES_Pin3;
		HAL_GPIO_Init(ES_PortB, &GPIO_InitStruct);
		HAL_GPIO_WritePin(ES_PortB, ES_Pin1, GPIO_PIN_SET); // Close the High Side of the GPIO to connect ES_Pin1 to Vdd
		HAL_GPIO_WritePin(ES_PortB, ES_Pin2, GPIO_PIN_SET); // Close the High Side of the GPIO to connect ES_Pin2 to Vdd
		HAL_GPIO_WritePin(ES_PortB, ES_Pin3, GPIO_PIN_SET); // Close the High Side of the GPIO to connect ES_Pin3 to Vdd
	}

	__HAL_RCC_GPIOA_CLK_DISABLE();
	__HAL_RCC_GPIOB_CLK_DISABLE();
	__HAL_RCC_GPIOC_CLK_DISABLE();
	__HAL_RCC_GPIOH_CLK_DISABLE();
}

void My_VDD_to_VDDS1_Switch(My_Switch_td mode)	// Opens and Closes the switches (High side of a GPIO) between Vdd and VDDS2
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Pin =  VDDS1_Pin;
	HAL_GPIO_Init(VDDS1_Port, &GPIO_InitStruct);

	if (mode == Switch_CLOSED)
	{
		HAL_GPIO_WritePin(VDDS1_Port, VDDS1_Pin, GPIO_PIN_SET); // Close the High Side of the GPIO to connect VDDS1 to Vdd
	}
	else
	{
		HAL_GPIO_WritePin(VDDS1_Port, VDDS1_Pin, GPIO_PIN_RESET); // OPen the High Side of the GPIO to connect VDDS1 to Vdd
	}

	__HAL_RCC_GPIOA_CLK_DISABLE();
	__HAL_RCC_GPIOB_CLK_DISABLE();
	__HAL_RCC_GPIOC_CLK_DISABLE();
	__HAL_RCC_GPIOH_CLK_DISABLE();
}

void My_VDD_to_VDDS2_Switch(My_Switch_td mode)	// Opens and Closes the switches (High side of a GPIO) between Vdd and VDDS2
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Pin =  VDDS2_Pin;
	HAL_GPIO_Init(VDDS2_Port, &GPIO_InitStruct);

	if (mode == Switch_CLOSED)
	{
		HAL_GPIO_WritePin(VDDS2_Port, VDDS2_Pin, GPIO_PIN_SET); // Close the High Side of the GPIO to connect VDDS2 to Vdd
	}
	else
	{
		HAL_GPIO_WritePin(VDDS2_Port, VDDS2_Pin, GPIO_PIN_RESET); // Close the High Side of the GPIO to connect VDDS2 to Vdd
	}

	__HAL_RCC_GPIOA_CLK_DISABLE();
	__HAL_RCC_GPIOB_CLK_DISABLE();
	__HAL_RCC_GPIOC_CLK_DISABLE();
	__HAL_RCC_GPIOH_CLK_DISABLE();
}
#endif

void My_HAL_Delay(__IO uint32_t Delay)
{
	/* TIMER_IF can be based on other counter the SysTick e.g. RTC */
	/* USER CODE BEGIN HAL_Delay_1 */

	/* USER CODE END HAL_Delay_1 */
	TIMER_IF_DelayMs(Delay);
	/* USER CODE BEGIN HAL_Delay_2 */

	/* USER CODE END HAL_Delay_2 */
}

void My_LPTIM1_Monitor(My_Switch_td mode)					// Opens and Closes the switches (High side of a GPIO) between Vdd and VDDS2
{
	__HAL_RCC_GPIOA_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Pin =  GPIO_PIN_0; 						// PA0 = CN10[1]
	HAL_GPIO_Init(VDDS1_Port, &GPIO_InitStruct);

	if (mode == Switch_CLOSED)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); // Close the High Side of the GPIO to connect VDDS1 to Vdd
	}
	else
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); // OPen the High Side of the GPIO to connect VDDS1 to Vdd
	}

	__HAL_RCC_GPIOA_CLK_DISABLE();
}

void My_RTCA_Monitor(My_Switch_td mode)	// Opens and Closes the switches (High side of a GPIO) between Vdd and VDDS2
{
	__HAL_RCC_GPIOC_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Pin =  GPIO_PIN_14; 	// PC14 = CN7[25]
	HAL_GPIO_Init(VDDS1_Port, &GPIO_InitStruct);

	if (mode == Switch_CLOSED)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET); // Close the High Side of the GPIO to connect VDDS1 to Vdd
	}
	else
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET); // OPen the High Side of the GPIO to connect VDDS1 to Vdd
	}

	__HAL_RCC_GPIOC_CLK_DISABLE();
}

void My_RTCB_Monitor(My_Switch_td mode)	// Opens and Closes the switches (High side of a GPIO) between Vdd and VDDS2
{
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Pin =  GPIO_PIN_13; 	// PB13 = CN7[38]
	HAL_GPIO_Init(VDDS1_Port, &GPIO_InitStruct);

	if (mode == Switch_CLOSED)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); // Close the High Side of the GPIO to connect VDDS1 to Vdd
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // OPen the High Side of the GPIO to connect VDDS1 to Vdd
	}

	__HAL_RCC_GPIOB_CLK_DISABLE();
}

uint32_t GetDeviceAddress(void)
{
	// Read the device address from flash memory
	uint32_t deviceAddress = *((uint32_t*)FLASH_DEVADDR_ADDRESS);
	return deviceAddress;
}

#if !MY_SYSTEM_INIT
void SystemInit(void)	// System Init Function
{

#if MY_DEBUGGER_ENABLED
	DBG_Init();
	SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_STOP);
	SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_SLEEP);
	SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_STANDBY);
#else
	//	My_Set_GPIO_ANALOG();							// Configure all GPIOs in Analog Mode (High Impedance)
#endif

	HAL_Init();										// Configure HAL

#if !(MY_DEBUGGER_ENABLED || MY_LPTIM_Monitor || MY_RTCB_Monitor)
	My_Set_All_GPIO_To_Analog_Mode();											// Set all GPIO in Analog Mode
#endif

	HAL_PWR_EnablePVD();							// Enable PVD

	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2); // Undervolting

	My_Set_PVD(V2P0, PWR_PVD_MODE_IT_RISING);		// Configure PVD
	HAL_PWREx_EnableLowPowerRunMode();				// Enter Low Power Run Mode
	HAL_NVIC_EnableIRQ(PVD_PVM_IRQn); 				// Enable PVD Interrupt
	HAL_PWREx_EnterSTOP0Mode(PWR_STOPENTRY_WFI);	// Enter STOP0 mode and WFI
	HAL_PWREx_DisableLowPowerRunMode();				// Disable Low Power Run Mode
}
#endif


