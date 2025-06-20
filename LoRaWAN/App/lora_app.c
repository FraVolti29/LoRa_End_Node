/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    lora_app.c
 * @author  MCD Application Team
 * @brief   Application of the LRWAN Middleware
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "platform.h"
#include "sys_app.h"
#include "lora_app.h"
#include "stm32_seq.h"
#include "stm32_timer.h"
#include "utilities_def.h"
#include "app_version.h"
#include "lorawan_version.h"
#include "subghz_phy_version.h"
#include "lora_info.h"
#include "LmHandler.h"
#include "adc_if.h"
#include "CayenneLpp.h"
#include "sys_sensors.h"
#include "flash_if.h"

/* USER CODE BEGIN Includes */
#include "my_lorawan.h"
#include "radio.h"
#include "LoRaMac.h"
#include "lorawan_conf.h"
#include "radio_def.h"
#include "main.h"
#include "app_x-cube-ai.h"
/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

uint8_t my_LoRAWAN_Init = 0;
#if MY_TX_OTAA
uint8_t my_Join_Accept_done = 0;
#endif

//extern uint8_t result_nn;

#if MY_SM_FEATURE_ENABLE
#if MY_SoilSensor
extern float my_SoilSensor;
#endif

#if MY_SHT40
extern uint32_t my_SHT40_Temp;
extern uint32_t my_SHT40_Humid;
#endif

#if MY_STTS22H
extern uint8_t my_STTS22H_Sensor_Flag;		// STTS22H Sensor Presence Flag
extern uint32_t my_STTS22H_Temp;				// STTS22H Ambient Temperature Value
#endif

#if MY_STHS34PF80
extern uint8_t my_STHS34PF80_Sensor_Flag;	// STHS34PF80 Sensor Presence Flag
extern uint8_t my_STHS34PF80_Motion;			// STHS34PF80 Motion Flag
extern uint8_t my_STHS34PF80_Presence;       	// STHS34PF80 Presence Flag
extern uint8_t my_STHS34PF80_Tamb_Shock;     	// STHS34PF80 Ambient Temperature Shock Flag

extern uint32_t my_STHS34PF80_Obj_Temp;        	// Object Temperature Value
extern uint32_t my_STHS34PF80_Amb_Temp;			// Ambient Temperature Value

extern uint16_t my_STHS34PF80_Presence_Data;
extern uint16_t my_STHS34PF80_Motion_Data;
#endif

#if MY_LIS2DU12
extern uint8_t my_LIS2DU12_Sensor_Flag;
extern uint16_t my_LIS2DU12_acc_x;
extern uint16_t my_LIS2DU12_acc_y;
extern uint16_t my_LIS2DU12_acc_z;
#endif

#endif
extern My_Mode_State_td my_Current_State;
extern My_Mode_State_td my_Previous_State;
extern My_Mode_State_td my_Next_State;
extern My_Cmode_td my_Cmode;

extern uint8_t my_ESD_Level;
extern uint8_t my_rm_failure_flag;
extern uint8_t My_EBK_status;
extern uint8_t my_ESS_cnt;

extern uint16_t my_Trise_time;
extern uint16_t my_Trise_time_start;
extern uint16_t my_Trise_time_stop;
extern uint16_t my_Tau_Resistance;
extern uint16_t my_Start_Timer;
extern uint16_t my_lptim_setup_value;
extern uint16_t my_lptim_value;
extern uint16_t my_ems_buffer[MY_DIM_EMS_BUFFER];

extern uint8_t my_N_LORA_TX;
#if MY_NOUSE
/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/**
 * @brief LoRa State Machine states
 */

typedef enum TxEventType_e
{
	/**
	 * @brief Appdata Transmission issue based on timer every TxDutyCycleTime
	 */
	TX_ON_TIMER,
	/**
	 * @brief Appdata Transmission external event plugged on OnSendEvent( )
	 */
	TX_ON_EVENT
	/* USER CODE BEGIN TxEventType_t */

	/* USER CODE END TxEventType_t */
} TxEventType_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/**
 * LEDs period value of the timer in ms
 */
#define LED_PERIOD_TIME 500

/**
 * Join switch period value of the timer in ms
 */
#define JOIN_TIME 2000
/*---------------------------------------------------------------------------*/
/*                             LoRaWAN NVM configuration                     */
/*---------------------------------------------------------------------------*/
/**
 * @brief LoRaWAN NVM Flash address
 * @note last 2 sector of a 128kBytes device
 */
#define LORAWAN_NVM_BASE_ADDRESS                    ((void *)0x0803F000UL)

/* USER CODE BEGIN PD */
static const char *slotStrings[] = { "1", "2", "C", "C_MC", "P", "P_MC" };

#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private function prototypes -----------------------------------------------*/
/**
 * @brief  LoRa End Node send request
 */

static void SendTxData(void);

/**
 * @brief  TX timer callback function
 * @param  context ptr of timer context
 */

#if MY_NOUSE
static void OnTxTimerEvent(void *context);
#endif

#if MY_TX_OTAA
/**
 *
 * @brief  join event callback function
 * @param  joinParams status of join
 */
static void OnJoinRequest(LmHandlerJoinParams_t *joinParams);
#endif

#if MY_TX_ABP
/**
 * @brief callback when LoRaWAN application has sent a frame
 * @brief  tx event callback function
 * @param  params status of last Tx
 */
static void OnTxData(LmHandlerTxParams_t *params);
#endif

#if MY_NOUSE

/**
 * @brief callback when LoRaWAN application has received a frame
 * @param appData data received in the last Rx
 * @param params status of last Rx
 */
static void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params);

/**
 * @brief callback when LoRaWAN Beacon status is updated
 * @param params status of Last Beacon
 */
static void OnBeaconStatusChange(LmHandlerBeaconParams_t *params);

/**
 * @brief callback when system time has been updated
 */
static void OnSysTimeUpdate(void);

/**
 * @brief callback when LoRaWAN application Class is changed
 * @param deviceClass new class
 */
static void OnClassChange(DeviceClass_t deviceClass);
/**
 * @brief  stop current LoRa execution to switch into non default Activation mode
 */

static void StopJoin(void);

/**
 * @brief  LoRa store context in Non Volatile Memory
 */
static void StoreContext(void);


/**
 * @brief  Join switch timer callback function
 * @param  context ptr of Join switch context
 */
static void OnStopJoinTimerEvent(void *context);
#endif


/**
 * @brief  Notifies the upper layer that the NVM context has changed
 * @param  state Indicates if we are storing (true) or restoring (false) the NVM context
 */
static void OnMacProcessNotify(void);



#if MY_NOUSE

/**
 * @brief Change the periodicity of the uplink frames
 * @param periodicity uplink frames period in ms
 * @note Compliance test protocol callbacks
 */

static void OnTxTimerLedEvent(void *context);

/**
 * @brief  LED Rx timer callback function
 * @param  context ptr of LED context
 */

static void OnNvmDataChange(LmHandlerNvmContextStates_t state);

/**
 * @brief  Store the NVM Data context to the Flash
 * @param  nvm ptr on nvm structure
 * @param  nvm_size number of data bytes which were stored
 */

static void OnStoreContextRequest(void *nvm, uint32_t nvm_size);

/**
 * @brief  Restore the NVM Data context from the Flash
 * @param  nvm ptr on nvm structure
 * @param  nvm_size number of data bytes which were restored
 */
static void OnRestoreContextRequest(void *nvm, uint32_t nvm_size);

/**
 * Will be called each time a Radio IRQ is handled by the MAC layer
 *
 */


static void OnTxPeriodicityChanged(uint32_t periodicity);

/**
 * @brief Change the confirmation control of the uplink frames
 * @param isTxConfirmed Indicates if the uplink requires an acknowledgement
 * @note Compliance test protocol callbacks
 */
static void OnTxFrameCtrlChanged(LmHandlerMsgTypes_t isTxConfirmed);

/**
 * @brief Change the periodicity of the ping slot frames
 * @param pingSlotPeriodicity ping slot frames period in ms
 * @note Compliance test protocol callbacks
 */
static void OnPingSlotPeriodicityChanged(uint8_t pingSlotPeriodicity);
#endif

#if MY_TX_OTAA
/**
 * @brief Will be called to reset the system
 * @note Compliance test protocol callbacks
 */
static void OnSystemReset(void);
#endif
/* USER CODE BEGIN PFP */
#if MY_NOUSE
/**
 * @brief  LED Tx timer callback function
 * @param  context ptr of LED context
 */

static void OnRxTimerLedEvent(void *context);

/**
 * @brief  LED Join timer callback function
 * @param  context ptr of LED context
 */
static void OnJoinTimerLedEvent(void *context);

/* USER CODE END PFP */
#endif


/* Private variables ---------------------------------------------------------*/

/**
 * @brief LoRaWAN default activation type
 */

static ActivationType_t ActivationType = LORAWAN_DEFAULT_ACTIVATION_TYPE;

/**
 * @brief LoRaWAN force rejoin even if the NVM context is restored
 */
static bool ForceRejoin = LORAWAN_FORCE_REJOIN_AT_BOOT;

/**
 * @brief LoRaWAN handler Callbacks
 */

#if MY_TX
static LmHandlerCallbacks_t LmHandlerCallbacks =
{
#if MY_TX_ABP
		.OnMacProcess =                 OnMacProcessNotify,
		.OnTxData =                     OnTxData,
#endif

#if MY_TX_OTAA
			.OnJoinRequest = OnJoinRequest,
			.GetUniqueId = GetUniqueId,
			.GetDevAddr = GetDevAddr,
			.OnMacProcess = OnMacProcessNotify,
			.OnSystemReset = OnSystemReset,
#endif
};
#endif

#if MY_NOUSE
static LmHandlerCallbacks_t LmHandlerCallbacks =
{
		.GetBatteryLevel =              GetBatteryLevel,
		.GetTemperature =               GetTemperatureLevel,
		.GetUniqueId =                  GetUniqueId,
		.GetDevAddr =                   GetDevAddr,
		.OnRestoreContextRequest =      OnRestoreContextRequest,
		.OnStoreContextRequest =        OnStoreContextRequest,
		.OnNvmDataChange =              OnNvmDataChange,
		.OnRxData =                     OnRxData,
		.OnBeaconStatusChange =         OnBeaconStatusChange,
		.OnSysTimeUpdate =              OnSysTimeUpdate,
		.OnClassChange =                OnClassChange,
		.OnTxPeriodicityChanged =       OnTxPeriodicityChanged,
		.OnTxFrameCtrlChanged =         OnTxFrameCtrlChanged,
		.OnPingSlotPeriodicityChanged = OnPingSlotPeriodicityChanged,
		.OnSystemReset =                OnSystemReset,
		.OnMacProcess =                 OnMacProcessNotify,
		.OnTxData =                     OnTxData,
		.OnJoinRequest =                OnJoinRequest,
		.OnMacProcess =                 OnMacProcessNotify,
		.OnTxData =                     OnTxData,
};

/**
 * @brief LoRaWAN handler parameters
 */
static LmHandlerParams_t LmHandlerParams =
{
		.ActiveRegion =             ACTIVE_REGION,
		.DefaultClass =             LORAWAN_DEFAULT_CLASS,
		.AdrEnable =                LORAWAN_ADR_STATE,
		.IsTxConfirmed =            LORAWAN_DEFAULT_CONFIRMED_MSG_STATE,
		.TxDatarate =               LORAWAN_DEFAULT_DATA_RATE,
		.TxPower =                  LORAWAN_DEFAULT_TX_POWER,
		.PingSlotPeriodicity =      LORAWAN_DEFAULT_PING_SLOT_PERIODICITY,
		.RxBCTimeout =              LORAWAN_DEFAULT_CLASS_B_C_RESP_TIMEOUT
};
#endif

#if MY_TX
static LmHandlerParams_t LmHandlerParams =
{
		.ActiveRegion =             ACTIVE_REGION,
		.DefaultClass =             LORAWAN_DEFAULT_CLASS,
		.AdrEnable =                LORAWAN_ADR_STATE,
		.IsTxConfirmed =            LORAWAN_DEFAULT_CONFIRMED_MSG_STATE,
		.TxDatarate =               LORAWAN_DEFAULT_DATA_RATE,
		.TxPower =                  LORAWAN_DEFAULT_TX_POWER,
};
#endif


#if MY_NOUSE

/**
 * @brief Type of Event to generate application Tx
 */

static TxEventType_t EventType = TX_ON_TIMER;

/**
 * @brief Timer to handle the application Tx
 */
static UTIL_TIMER_Object_t TxTimer;

/**
 * @brief Tx Timer period
 */
static UTIL_TIMER_Time_t TxPeriodicity = APP_TX_DUTYCYCLE;

/**
 * @brief Join Timer period
 */
static UTIL_TIMER_Object_t StopJoinTimer;
#endif

/* USER CODE BEGIN PV */
uint8_t my_SubghzApp_Init_done = 0;
/**
 * @brief User application buffer
 */
static uint8_t AppDataBuffer[LORAWAN_APP_DATA_BUFFER_MAX_SIZE];

/**
 * @brief User application data structure
 */
static LmHandlerAppData_t AppData = { 0, 0, AppDataBuffer };

#if MY_NOUSE
/**
 * @brief Specifies the state of the application LED
 */
static uint8_t AppLedStateOn = RESET;
/**
 * @brief Timer to handle the application Tx Led to toggle
 */
static UTIL_TIMER_Object_t TxLedTimer;

/**
 * @brief Timer to handle the application Rx Led to toggle
 */
static UTIL_TIMER_Object_t RxLedTimer;

/**
 * @brief Timer to handle the application Join Led to toggle
 */

static UTIL_TIMER_Object_t JoinLedTimer;
#endif

/* USER CODE END PV */

/* Exported functions ---------------------------------------------------------*/
/* USER CODE BEGIN EF */


/* USER CODE END EF */

void LoRaWAN_Init(void)
{
	/* USER CODE BEGIN LoRaWAN_Init_LV */

#if MY_PRINT
	uint32_t feature_version = 0UL;
	/* USER CODE END LoRaWAN_Init_LV */

	/* USER CODE BEGIN LoRaWAN_Init_1 */

	/* Get LoRaWAN APP version*/
	APP_LOG(TS_OFF, VLEVEL_M, "APPLICATION_VERSION: V%X.%X.%X\r\n",
			(uint8_t)(APP_VERSION_MAIN),
			(uint8_t)(APP_VERSION_SUB1),
			(uint8_t)(APP_VERSION_SUB2));

	/* Get MW LoRaWAN info */
	APP_LOG(TS_OFF, VLEVEL_M, "MW_LORAWAN_VERSION:  V%X.%X.%X\r\n",
			(uint8_t)(LORAWAN_VERSION_MAIN),
			(uint8_t)(LORAWAN_VERSION_SUB1),
			(uint8_t)(LORAWAN_VERSION_SUB2));

	/* Get MW SubGhz_Phy info */
	APP_LOG(TS_OFF, VLEVEL_M, "MW_RADIO_VERSION:    V%X.%X.%X\r\n",
			(uint8_t)(SUBGHZ_PHY_VERSION_MAIN),
			(uint8_t)(SUBGHZ_PHY_VERSION_SUB1),
			(uint8_t)(SUBGHZ_PHY_VERSION_SUB2));

	/* Get LoRaWAN Link Layer info */
	LmHandlerGetVersion(LORAMAC_HANDLER_L2_VERSION, &feature_version);
	APP_LOG(TS_OFF, VLEVEL_M, "L2_SPEC_VERSION:     V%X.%X.%X\r\n",
			(uint8_t)(feature_version >> 24),
			(uint8_t)(feature_version >> 16),
			(uint8_t)(feature_version >> 8));

	/* Get LoRaWAN Regional Parameters info */
	LmHandlerGetVersion(LORAMAC_HANDLER_REGION_VERSION, &feature_version);
	APP_LOG(TS_OFF, VLEVEL_M, "RP_SPEC_VERSION:     V%X-%X.%X.%X\r\n",
			(uint8_t)(feature_version >> 24),
			(uint8_t)(feature_version >> 16),
			(uint8_t)(feature_version >> 8),
			(uint8_t)(feature_version));
	UTIL_TIMER_Create(&TxLedTimer, LED_PERIOD_TIME, UTIL_TIMER_ONESHOT, OnTxTimerLedEvent, NULL);
	UTIL_TIMER_Create(&RxLedTimer, LED_PERIOD_TIME, UTIL_TIMER_ONESHOT, OnRxTimerLedEvent, NULL);
	UTIL_TIMER_Create(&JoinLedTimer, LED_PERIOD_TIME, UTIL_TIMER_PERIODIC, OnJoinTimerLedEvent, NULL);

	if (FLASH_IF_Init(NULL) != FLASH_IF_OK)
	{
		Error_Handler();
	}
	/* USER CODE END LoRaWAN_Init_1 */

	UTIL_TIMER_Create(&StopJoinTimer, JOIN_TIME, UTIL_TIMER_ONESHOT, OnStopJoinTimerEvent, NULL);
	UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LmHandlerProcess), UTIL_SEQ_RFU, LmHandlerProcess);
	UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), UTIL_SEQ_RFU, SendTxData);
	UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaStoreContextEvent), UTIL_SEQ_RFU, StoreContext);
	UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaStopJoinEvent), UTIL_SEQ_RFU, StopJoin);

	/* Init Info table used by LmHandler*/
	LoraInfo_Init();

	/* Init the Lora Stack*/
	LmHandlerInit(&LmHandlerCallbacks, APP_VERSION);

	LmHandlerConfigure(&LmHandlerParams);

	/* USER CODE BEGIN LoRaWAN_Init_2 */

	UTIL_TIMER_Start(&JoinLedTimer);


	/* USER CODE END LoRaWAN_Init_2 */

	LmHandlerJoin(ActivationType, ForceRejoin);

	if (EventType == TX_ON_TIMER)
	{
		/* send every time timer elapses */
		UTIL_TIMER_Create(&TxTimer, TxPeriodicity, UTIL_TIMER_ONESHOT, OnTxTimerEvent, NULL);
		UTIL_TIMER_Start(&TxTimer);
	}
	else
	{
		/* USER CODE BEGIN LoRaWAN_Init_3 */

		/* USER CODE END LoRaWAN_Init_3 */
	}

	/* USER CODE BEGIN LoRaWAN_Init_Last */
#endif

#if MY_TX

#if MY_TX_ABP
	UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_RTF), UTIL_SEQ_RFU, SendTxData);	// Register task for: SendTxData
	UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_EHF), UTIL_SEQ_RFU, My_EHF);		// Register task for: My_EHF
	UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_ESF), UTIL_SEQ_RFU, My_ESF);		// Register task for: My_ESF
	UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_SMF), UTIL_SEQ_RFU, My_SMF);	 	// Register Task for: My_SMF
#if AI
	UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_AIF), UTIL_SEQ_RFU, My_AIF);		// Register Task for: My_AIF
#endif

#endif

#if MY_TX_OTAA
	UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LmHandlerProcess), UTIL_SEQ_RFU, LmHandlerProcess); // Register task for: LmHandlerProcess
	UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_RTF), UTIL_SEQ_RFU, SendTxData);	// Register task for: SendTxData
	UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_EHF), UTIL_SEQ_RFU, My_EHF);		// Register task for: My_EHF
	UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_ESF), UTIL_SEQ_RFU, My_ESF);		// Register task for: My_ESF
	UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_SMF), UTIL_SEQ_RFU, My_SMF);	 	// Register Task for: My_SMF
#endif

	LoraInfo_Init();									 					// LoRaWAN info (Region, ClassB, Kms(Key Management Service))

	LmHandlerInit(&LmHandlerCallbacks, APP_VERSION);     					// LoRaMacPrimitives: Used to notify LmHandler of LoRaMac events and LoRaMacCallbacks On MacProcessNotify:
	// When a radio interrupt (IRQ) occurs, it means that the radio has finished transmitting or receiving a message and needs to be handled by the MAC layer

	//LmHandlerParams.TxDatarate = 2;
	LmHandlerConfigure(&LmHandlerParams);                					// LoRaWAN Configuration using LmHandlerParams

	LmHandlerJoin(ActivationType, ForceRejoin);          					// LoRaWAN network ACTIVATION_TYPE_ABP file  (LmHandlerJoin) is modified, to work only with ABP no need to OTAA activation

	if (my_Current_State != PSS)
	{
		my_SubghzApp_Init_done = 1;

#if MY_RADIO_DEACTIVATION
		Radio.Sleep();
#endif
	}
#if MY_TX_ABP
	SendTxData();
#endif
	my_LoRAWAN_Init++;
#endif
	/* USER CODE END LoRaWAN_Init_Last */
}

#if MY_NOUSE
/* USER CODE BEGIN PB_Callbacks */


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch (GPIO_Pin)
	{
	case  BUT1_Pin:
		/* Note: when "EventType == TX_ON_TIMER" this GPIO is not initialized */
		if (EventType == TX_ON_EVENT)
		{
			UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), CFG_SEQ_Prio_0);
		}
		break;
	case  BUT2_Pin:
		UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaStopJoinEvent), CFG_SEQ_Prio_0);
		break;
	case  BUT3_Pin:
		UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaStoreContextEvent), CFG_SEQ_Prio_0);
		break;
	default:
		break;
	}
}

/* USER CODE END PB_Callbacks */

/* Private functions ---------------------------------------------------------*/
/* USER CODE BEGIN PrFD */

/* USER CODE END PrFD */
static void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params)
{
	/* USER CODE BEGIN OnRxData_1 */
	uint8_t RxPort = 0;

	if (params != NULL)
	{
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); /* LED_BLUE */

		UTIL_TIMER_Start(&RxLedTimer);

		if (params->IsMcpsIndication)
		{
			if (appData != NULL)
			{
				RxPort = appData->Port;
				if (appData->Buffer != NULL)
				{
					switch (appData->Port)
					{
					case LORAWAN_SWITCH_CLASS_PORT:
						/*this port switches the class*/
						if (appData->BufferSize == 1)
						{
							switch (appData->Buffer[0])
							{
							case 0:
							{
								LmHandlerRequestClass(CLASS_A);
								break;
							}
							case 1:
							{
								LmHandlerRequestClass(CLASS_B);
								break;
							}
							case 2:
							{
								LmHandlerRequestClass(CLASS_C);
								break;
							}
							default:
								break;
							}
						}
						break;
					case LORAWAN_USER_APP_PORT:
						if (appData->BufferSize == 1)
						{
							AppLedStateOn = appData->Buffer[0] & 0x01;
							if (AppLedStateOn == RESET)
							{
								APP_LOG(TS_OFF, VLEVEL_H, "LED OFF\r\n");
								HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET); /* LED_RED */
							}
							else
							{
								APP_LOG(TS_OFF, VLEVEL_H, "LED ON\r\n");
								HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET); /* LED_RED */
							}
						}
						break;

					default:

						break;
					}
				}
			}
		}
		if (params->RxSlot < RX_SLOT_NONE)
		{
			APP_LOG(TS_OFF, VLEVEL_H, "###### D/L FRAME:%04d | PORT:%d | DR:%d | SLOT:%s | RSSI:%d | SNR:%d\r\n",
					params->DownlinkCounter, RxPort, params->Datarate, slotStrings[params->RxSlot],
					params->Rssi, params->Snr);
		}
	}
	/* USER CODE END OnRxData_1 */
}
#endif


#if MY_TX

void My_Adv_Data_Init(void)
{
	uint32_t i = 0;

	if (my_LoRAWAN_Init++ > N_LORA_TX)
	{
//		AppData.Buffer[i++] = (uint8_t)((my_Trise_time >> 8) & 0xFF); 			// 0
//		AppData.Buffer[i++] = (uint8_t)(my_Trise_time & 0xFF); 					// 1

#if MY_ES_FEATURE_ENABLE

		if(my_Cmode == DCM)
		{
			my_ESD_Level = 0;
		}
		else if (my_Cmode == CCM)
		{
			my_ESD_Level = 1;
		}
		else
		{
			my_ESD_Level = 2;
		}
		AppData.Buffer[i++] = (uint8_t)(my_ESD_Level & 0xFF); 					// 2
		AppData.Buffer[i++] = (uint8_t)(My_EBK_status & 0xFF); 					// 3

#endif

#if MY_SM_FEATURE_ENABLE

#if MY_SHT40
		/**** Get Temperature from STH40 ****/
		AppData.Buffer[i++] = (uint8_t)((my_SHT40_Temp >> 24) & 0xFF); 			// 4
		AppData.Buffer[i++] = (uint8_t)((my_SHT40_Temp >> 16) & 0xFF); 			// 5
		AppData.Buffer[i++] = (uint8_t)((my_SHT40_Temp >> 8) & 0xFF); 			// 6
		AppData.Buffer[i++] = (uint8_t)(my_SHT40_Temp & 0xFF); 					// 7

		/**** Get RH from STH40 ****/
		AppData.Buffer[i++] = (uint8_t)((my_SHT40_Humid >> 24) & 0xFF); 		// 8
		AppData.Buffer[i++] = (uint8_t)((my_SHT40_Humid >> 16) & 0xFF); 		// 9
		AppData.Buffer[i++] = (uint8_t)((my_SHT40_Humid >> 8) & 0xFF); 			// 10
		AppData.Buffer[i++] = (uint8_t)(my_SHT40_Humid & 0xFF); 				// 11
#endif

#if MY_STTS22H

		AppData.Buffer[i++] = (uint8_t)(my_STTS22H_Sensor_Flag & 0xFF);		 	// 12

		/**** Get Temperature from STTS22H ****/
		AppData.Buffer[i++] = (uint8_t)((my_STTS22H_Temp >> 24) & 0xFF); 		// 13
		AppData.Buffer[i++] = (uint8_t)((my_STTS22H_Temp >> 16) & 0xFF); 		// 14
		AppData.Buffer[i++] = (uint8_t)((my_STTS22H_Temp >> 8) & 0xFF); 		// 15
		AppData.Buffer[i++] = (uint8_t)(my_STTS22H_Temp & 0xFF); 				// 16
#endif

#if MY_STHS34PF80

		AppData.Buffer[i++] = (my_STHS34PF80_Sensor_Flag & 0xFF);				// 17

		/**** Get Motion Flag from STHS34PF80 ****/
		AppData.Buffer[i++] = (my_STHS34PF80_Motion & 0xFF); 					// 18

		/**** Get Presence Flag from STHS34PF80 ****/
		AppData.Buffer[i++] = (my_STHS34PF80_Presence & 0xFF); 					// 19

		/**** Get Ambient Temperature Shock Flag from STHS34PF80 ****/
		AppData.Buffer[i++] = (my_STHS34PF80_Tamb_Shock & 0xFF); 				// 20

		/**** Get Object Temperature from STHS34PF80 ****/
		AppData.Buffer[i++] = (uint8_t)((my_STHS34PF80_Obj_Temp >> 24) & 0xFF);	// 21
		AppData.Buffer[i++] = (uint8_t)((my_STHS34PF80_Obj_Temp >> 16) & 0xFF);	// 22
		AppData.Buffer[i++] = (uint8_t)((my_STHS34PF80_Obj_Temp >> 8) & 0xFF); 	// 23
		AppData.Buffer[i++] = (uint8_t)(my_STHS34PF80_Obj_Temp & 0xFF); 		// 24

		/**** Get Objact Temperature from STHS34PF80 ****/
		AppData.Buffer[i++] = (uint8_t)((my_STHS34PF80_Amb_Temp >> 24) & 0xFF);	// 25
		AppData.Buffer[i++] = (uint8_t)((my_STHS34PF80_Amb_Temp >> 16) & 0xFF);	// 26
		AppData.Buffer[i++] = (uint8_t)((my_STHS34PF80_Amb_Temp >> 8) & 0xFF); 	// 27
		AppData.Buffer[i++] = (uint8_t)(my_STHS34PF80_Amb_Temp & 0xFF); 		// 28

		AppData.Buffer[i++] = (uint8_t)((my_STHS34PF80_Presence_Data >> 8) & 0xFF); // 29
		AppData.Buffer[i++] = (uint8_t)(my_STHS34PF80_Presence_Data & 0xFF); 		// 30

		AppData.Buffer[i++] = (uint8_t)((my_STHS34PF80_Motion_Data >> 8) & 0xFF); 	// 31
		AppData.Buffer[i++] = (uint8_t)(my_STHS34PF80_Motion_Data & 0xFF); 			// 32
#endif

#if MY_LIS2DU12
		AppData.Buffer[i++] = (my_LIS2DU12_Sensor_Flag & 0XFF);						// 29

		AppData.Buffer[i++] = (uint8_t)((my_LIS2DU12_acc_x >> 8) & 0xFF);			// 30
		AppData.Buffer[i++] = (uint8_t)(my_LIS2DU12_acc_x & 0xFF);					// 31

		AppData.Buffer[i++] = (uint8_t)((my_LIS2DU12_acc_y >> 8) & 0xFF);			// 32
		AppData.Buffer[i++] = (uint8_t)(my_LIS2DU12_acc_y & 0xFF);					// 33

		AppData.Buffer[i++] = (uint8_t)((my_LIS2DU12_acc_z >> 8) & 0xFF);			// 34
		AppData.Buffer[i++] = (uint8_t)(my_LIS2DU12_acc_z & 0xFF);					// 35
#endif

#endif

#if 0
		/**** Get Sensor IDs ****/
		AppData.Buffer[i++] =(uint8_t)(sht40_id & 0xFF);							// 37
		AppData.Buffer[i++] =(uint8_t)(stts22h_id & 0xFF);							// 38
		AppData.Buffer[i++] =(uint8_t)(sths34pf_id & 0xFF);							// 39
		AppData.Buffer[i++] =(uint8_t)(lis2du12_id & 0xFF);							// 40
#endif

#if 0
		AppData.Buffer[i++] = (uint8_t)((my_Trise_time_start >> 8) & 0xFF); 	// 41
		AppData.Buffer[i++] = (uint8_t)(my_Trise_time_start & 0xFF); 			// 42

		AppData.Buffer[i++] = (uint8_t)((my_Trise_time_stop >> 8) & 0xFF); 		// 43
		AppData.Buffer[i++] = (uint8_t)(my_Trise_time_stop & 0xFF); 			// 44
#endif
		/* Send the result of the neural network*/
#if AI
		AppData.Buffer[i++] = (uint8_t)(my_SoilSensor);
		AppData.Buffer[i++] = (uint8_t)(result_nn);
#endif
	}

	AppData.BufferSize = i;
	AppData.Port = LORAWAN_USER_APP_PORT;
	my_Trise_time = 0;
}

void My_EMS_Adv_Data_Init(void)
{
	uint32_t i = 0;
	uint8_t j = 0;

	for(j=0;j<MY_DIM_EMS_BUFFER;j++)
	{
		AppData.Buffer[i++] = (uint8_t)((my_ems_buffer[j] >> 8) & 0xFF);
		AppData.Buffer[i++] = (uint8_t)(my_ems_buffer[j] & 0xFF);
		my_ems_buffer[j] = 0;
	}
	AppData.BufferSize = i;
	AppData.Port = LORAWAN_USER_APP_PORT;

}

void SendTxData(void)
{
#if !(MY_DEBUGGER_ENABLED || MY_LPTIM_Monitor || MY_RTCB_Monitor)
	My_Set_All_GPIO_To_Analog_Mode();								// Set all GPIO in Analog Mode
#endif

#if MY_ES_FEATURE_ENABLE
	My_VDD_to_ES_Switch(Switch_OPEN);											// Open VDD to ES Switch
#endif

#if MY_LIS2DU12 && STEVAL_HARVEST1
	My_VDD_to_VDDS1_Switch(Switch_OPEN);										// unBias the Sensors
#endif

#if MY_SHT40 && MY_STHS34PF80 && MY_STTS22H && STEVAL_HARVEST1
	My_VDD_to_VDDS2_Switch(Switch_OPEN);										// unBias the Sensors
#endif

	HAL_NVIC_SetPriority(SUBGHZ_Radio_IRQn, 0, 0); 	// Configure Radio Interrupt with the Highest Priority
	HAL_NVIC_EnableIRQ(SUBGHZ_Radio_IRQn);			// Set Radio Interrupt

	if (my_Current_State == PSS)
	{
		HAL_ResumeTick();
		while(1)
		{
			My_Adv_Data_Init();
			HAL_Delay(3000);			// If Supplied by a Battery --> Add a delay for the Idle time between TX
			my_Previous_State = PSS;
			LmHandlerSend(&AppData, LmHandlerParams.IsTxConfirmed, false);	// send the data using LoRaWAN parameters (lmhandler params) with zero delay on Tx
		}
	}
	else
	{
		My_Exit_Stop2_Mode_WFI();

		HAL_NVIC_ClearPendingIRQ(LPTIM1_IRQn);	// Clear LPTIM1 Interrupt
		HAL_NVIC_DisableIRQ(LPTIM1_IRQn);		// Disable LPTIM1 Interrupt

		if (my_Current_State != SMS)
		{
			my_Previous_State = EHS;
			my_Current_State = RTS;				// EHS --> RTS
		}

		/*Check if it is the first Wake-up event after the Energy Missing State (EMS)
		 *If the system just woke up from EMS it sends the buffer that has been filled during the night
		 */
		if(my_Previous_State == EMS)
		{
			My_EMS_Adv_Data_Init();
		}
		else 									// Otherwise it sends the last measurement
		{
			My_Adv_Data_Init();
		}

		BSP_RADIO_Init();
		LmHandlerSend(&AppData, LmHandlerParams.IsTxConfirmed, false);	// send the data using LoRaWAN parameters (lmhandler params) with zero delay on Tx
		BSP_RADIO_DeInit();
	}
}
#endif


#if MY_NOUSE
static void SendTxData(void)
{
	/* USER CODE BEGIN SendTxData_1 */
	LmHandlerErrorStatus_t status = LORAMAC_HANDLER_ERROR;
	uint8_t batteryLevel = GetBatteryLevel();
	sensor_t sensor_data;
	UTIL_TIMER_Time_t nextTxIn = 0;

	if (LmHandlerIsBusy() == false)
	{
#ifdef CAYENNE_LPP
		uint8_t channel = 0;
#else
		uint16_t pressure = 0;
		int16_t temperature = 0;
		uint16_t humidity = 0;
		uint32_t i = 0;
		int32_t latitude = 0;
		int32_t longitude = 0;
		uint16_t altitudeGps = 0;
#endif /* CAYENNE_LPP */

		EnvSensors_Read(&sensor_data);

		APP_LOG(TS_ON, VLEVEL_M, "VDDA: %d\r\n", batteryLevel);
		APP_LOG(TS_ON, VLEVEL_M, "temp: %d\r\n", (int16_t)(sensor_data.temperature));

		AppData.Port = LORAWAN_USER_APP_PORT;

#ifdef CAYENNE_LPP
		CayenneLppReset();
		CayenneLppAddBarometricPressure(channel++, sensor_data.pressure);
		CayenneLppAddTemperature(channel++, sensor_data.temperature);
		CayenneLppAddRelativeHumidity(channel++, (uint16_t)(sensor_data.humidity));

		if ((LmHandlerParams.ActiveRegion != LORAMAC_REGION_US915) && (LmHandlerParams.ActiveRegion != LORAMAC_REGION_AU915)
				&& (LmHandlerParams.ActiveRegion != LORAMAC_REGION_AS923))
		{
			CayenneLppAddDigitalInput(channel++, GetBatteryLevel());
			CayenneLppAddDigitalOutput(channel++, AppLedStateOn);
		}

		CayenneLppCopy(AppData.Buffer);
		AppData.BufferSize = CayenneLppGetSize();
#else  /* not CAYENNE_LPP */
		humidity    = (uint16_t)(sensor_data.humidity * 10);            /* in %*10     */
		temperature = (int16_t)(sensor_data.temperature);
		pressure = (uint16_t)(sensor_data.pressure * 100 / 10); /* in hPa / 10 */

		AppData.Buffer[i++] = AppLedStateOn;
		AppData.Buffer[i++] = (uint8_t)((pressure >> 8) & 0xFF);
		AppData.Buffer[i++] = (uint8_t)(pressure & 0xFF);
		AppData.Buffer[i++] = (uint8_t)(temperature & 0xFF);
		AppData.Buffer[i++] = (uint8_t)((humidity >> 8) & 0xFF);
		AppData.Buffer[i++] = (uint8_t)(humidity & 0xFF);

		if ((LmHandlerParams.ActiveRegion == LORAMAC_REGION_US915) || (LmHandlerParams.ActiveRegion == LORAMAC_REGION_AU915)
				|| (LmHandlerParams.ActiveRegion == LORAMAC_REGION_AS923))
		{
			AppData.Buffer[i++] = 0;
			AppData.Buffer[i++] = 0;
			AppData.Buffer[i++] = 0;
			AppData.Buffer[i++] = 0;
		}
		else
		{
			latitude = sensor_data.latitude;
			longitude = sensor_data.longitude;

			AppData.Buffer[i++] = GetBatteryLevel();        /* 1 (very low) to 254 (fully charged) */
			AppData.Buffer[i++] = (uint8_t)((latitude >> 16) & 0xFF);
			AppData.Buffer[i++] = (uint8_t)((latitude >> 8) & 0xFF);
			AppData.Buffer[i++] = (uint8_t)(latitude & 0xFF);
			AppData.Buffer[i++] = (uint8_t)((longitude >> 16) & 0xFF);
			AppData.Buffer[i++] = (uint8_t)((longitude >> 8) & 0xFF);
			AppData.Buffer[i++] = (uint8_t)(longitude & 0xFF);
			AppData.Buffer[i++] = (uint8_t)((altitudeGps >> 8) & 0xFF);
			AppData.Buffer[i++] = (uint8_t)(altitudeGps & 0xFF);
		}

		AppData.BufferSize = i;
#endif /* CAYENNE_LPP */

		if ((JoinLedTimer.IsRunning) && (LmHandlerJoinStatus() == LORAMAC_HANDLER_SET))
		{
			UTIL_TIMER_Stop(&JoinLedTimer);
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET); /* LED_RED */
		}

		status = LmHandlerSend(&AppData, LmHandlerParams.IsTxConfirmed, false);
		if (LORAMAC_HANDLER_SUCCESS == status)
		{
			APP_LOG(TS_ON, VLEVEL_L, "SEND REQUEST\r\n");
		}
		else if (LORAMAC_HANDLER_DUTYCYCLE_RESTRICTED == status)
		{
			nextTxIn = LmHandlerGetDutyCycleWaitTime();
			if (nextTxIn > 0)
			{
				APP_LOG(TS_ON, VLEVEL_L, "Next Tx in  : ~%d second(s)\r\n", (nextTxIn / 1000));
			}
		}
	}

	if (EventType == TX_ON_TIMER)
	{
		UTIL_TIMER_Stop(&TxTimer);
		UTIL_TIMER_SetPeriod(&TxTimer, MAX(nextTxIn, TxPeriodicity));
		UTIL_TIMER_Start(&TxTimer);
	}

	/* USER CODE END SendTxData_1 */
}

static void OnTxTimerEvent(void *context)
{
	/* USER CODE BEGIN OnTxTimerEvent_1 */

	/* USER CODE END OnTxTimerEvent_1 */

	UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), CFG_SEQ_Prio_0);
	/*Wait for next tx slot*/
	UTIL_TIMER_Start(&TxTimer);

	/* USER CODE BEGIN OnTxTimerEvent_2 */

	/* USER CODE END OnTxTimerEvent_2 */
}

/* USER CODE BEGIN PrFD_LedEvents */
static void OnTxTimerLedEvent(void *context)
{
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET); /* LED_GREEN */
}

static void OnRxTimerLedEvent(void *context)
{
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET); /* LED_BLUE */
}

static void OnJoinTimerLedEvent(void *context)
{
	HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin); /* LED_RED */
}

/* USER CODE END PrFD_LedEvents */

static void OnTxData(LmHandlerTxParams_t *params)
{
	/* USER CODE BEGIN OnTxData_1 */
	if ((params != NULL))
	{
		/* Process Tx event only if its a mcps response to prevent some internal events (mlme) */
		if (params->IsMcpsConfirm != 0)
		{
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET); /* LED_GREEN */
			UTIL_TIMER_Start(&TxLedTimer);

			APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### ========== MCPS-Confirm =============\r\n");
			APP_LOG(TS_OFF, VLEVEL_H, "###### U/L FRAME:%04d | PORT:%d | DR:%d | PWR:%d", params->UplinkCounter,
					params->AppData.Port, params->Datarate, params->TxPower);

			APP_LOG(TS_OFF, VLEVEL_H, " | MSG TYPE:");
			if (params->MsgType == LORAMAC_HANDLER_CONFIRMED_MSG)
			{
				APP_LOG(TS_OFF, VLEVEL_H, "CONFIRMED [%s]\r\n", (params->AckReceived != 0) ? "ACK" : "NACK");
			}
			else
			{
				APP_LOG(TS_OFF, VLEVEL_H, "UNCONFIRMED\r\n");
			}
		}
	}
	/* USER CODE END OnTxData_1 */

}
#endif
#if MY_TX_ABP
static void OnTxData(LmHandlerTxParams_t *params)
{
	/* USER CODE BEGIN OnTxData_1 */

	/* USER CODE END OnTxData_1 */
}
#endif

#if MY_TX_OTAA
static void OnJoinRequest(LmHandlerJoinParams_t *joinParams)
{
	/* USER CODE BEGIN OnJoinRequest_1 */
	if (joinParams != NULL)
	{
		if (joinParams->Status == LORAMAC_HANDLER_SUCCESS)
		{

			if (joinParams->Mode == ACTIVATION_TYPE_OTAA)
			{
				my_Join_Accept_done = 1;
				my_Current_State  = EHS;
				my_Next_State = RTS;
				My_RTC_Init();

			}
		}
	}
	/* USER CODE END OnJoinRequest_1 */
}
#endif

#if MY_NOUSE

static void OnJoinRequest(LmHandlerJoinParams_t *joinParams)
{
	/* USER CODE BEGIN OnJoinRequest_1 */
	if (joinParams != NULL)
	{
		if (joinParams->Status == LORAMAC_HANDLER_SUCCESS)
		{
			UTIL_TIMER_Stop(&JoinLedTimer);
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET); /* LED_RED */

			APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### = JOINED = ");
			if (joinParams->Mode == ACTIVATION_TYPE_ABP)
			{
				APP_LOG(TS_OFF, VLEVEL_M, "ABP ======================\r\n");
			}
			else
			{
				APP_LOG(TS_OFF, VLEVEL_M, "OTAA =====================\r\n");
			}
		}
		else
		{
			APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### = JOIN FAILED\r\n");
		}

		APP_LOG(TS_OFF, VLEVEL_H, "###### U/L FRAME:JOIN | DR:%d | PWR:%d\r\n", joinParams->Datarate, joinParams->TxPower);
	}
	/* USER CODE END OnJoinRequest_1 */
}


static void OnBeaconStatusChange(LmHandlerBeaconParams_t *params)
{
	/* USER CODE BEGIN OnBeaconStatusChange_1 */
	if (params != NULL)
	{
		switch (params->State)
		{
		default:
		case LORAMAC_HANDLER_BEACON_LOST:
		{
			APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### BEACON LOST\r\n");
			break;
		}
		case LORAMAC_HANDLER_BEACON_RX:
		{
			APP_LOG(TS_OFF, VLEVEL_M,
					"\r\n###### BEACON RECEIVED | DR:%d | RSSI:%d | SNR:%d | FQ:%d | TIME:%d | DESC:%d | "
					"INFO:02X%02X%02X %02X%02X%02X\r\n",
					params->Info.Datarate, params->Info.Rssi, params->Info.Snr, params->Info.Frequency,
					params->Info.Time.Seconds, params->Info.GwSpecific.InfoDesc,
					params->Info.GwSpecific.Info[0], params->Info.GwSpecific.Info[1],
					params->Info.GwSpecific.Info[2], params->Info.GwSpecific.Info[3],
					params->Info.GwSpecific.Info[4], params->Info.GwSpecific.Info[5]);
			break;
		}
		case LORAMAC_HANDLER_BEACON_NRX:
		{
			APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### BEACON NOT RECEIVED\r\n");
			break;
		}
		}
	}
	/* USER CODE END OnBeaconStatusChange_1 */
}

static void OnSysTimeUpdate(void)
{
	/* USER CODE BEGIN OnSysTimeUpdate_1 */

	/* USER CODE END OnSysTimeUpdate_1 */
}

static void OnClassChange(DeviceClass_t deviceClass)
{
	/* USER CODE BEGIN OnClassChange_1 */
	APP_LOG(TS_OFF, VLEVEL_M, "Switch to Class %c done\r\n", "ABC"[deviceClass]);
	/* USER CODE END OnClassChange_1 */
}
#endif

#if MY_TX
static void OnMacProcessNotify(void)
{
#if MY_TX_ABP
	/* USER CODE BEGIN OnMacProcessNotify_1 */
	LoRaMacProcess( );                                                            // should be Called at first the LoRaMAC process before to run all package process features

	/* USER CODE END OnMacProcessNotify_1 */

	/* USER CODE BEGIN OnMacProcessNotify_2 */
	UTIL_SEQ_SetTask((1 <<CFG_SEQ_Task_EHF ), CFG_SEQ_Prio_0);   // My_EHF
#endif
#if MY_TX_OTAA
	if(!my_Join_Accept_done){
		UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LmHandlerProcess), CFG_SEQ_Prio_0); // LmHandlerProcess
		}
	/* USER CODE BEGIN OnMacProcessNotify_2 */
	if(my_Join_Accept_done){
		LoRaMacProcess( );
		UTIL_SEQ_SetTask((1 <<CFG_SEQ_Task_EHF ), CFG_SEQ_Prio_0);   // My_EHF
		}
	/* USER CODE END OnMacProcessNotify_2 */
#endif
	/* USER CODE END OnMacProcessNotify_2 */
}
#endif

#if MY_NOUSE
static void OnMacProcessNotify(void)
{
	/* USER CODE BEGIN OnMacProcessNotify_1 */

	/* USER CODE END OnMacProcessNotify_1 */
	UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LmHandlerProcess), CFG_SEQ_Prio_0);

	/* USER CODE BEGIN OnMacProcessNotify_2 */

	/* USER CODE END OnMacProcessNotify_2 */
}

static void OnTxPeriodicityChanged(uint32_t periodicity)
{
	/* USER CODE BEGIN OnTxPeriodicityChanged_1 */

	/* USER CODE END OnTxPeriodicityChanged_1 */
	TxPeriodicity = periodicity;

	if (TxPeriodicity == 0)
	{
		/* Revert to application default periodicity */
		TxPeriodicity = APP_TX_DUTYCYCLE;
	}

	/* Update timer periodicity */
	UTIL_TIMER_Stop(&TxTimer);
	UTIL_TIMER_SetPeriod(&TxTimer, TxPeriodicity);
	UTIL_TIMER_Start(&TxTimer);
	/* USER CODE BEGIN OnTxPeriodicityChanged_2 */

	/* USER CODE END OnTxPeriodicityChanged_2 */
}

static void OnTxFrameCtrlChanged(LmHandlerMsgTypes_t isTxConfirmed)
{
	/* USER CODE BEGIN OnTxFrameCtrlChanged_1 */

	/* USER CODE END OnTxFrameCtrlChanged_1 */
	LmHandlerParams.IsTxConfirmed = isTxConfirmed;
	/* USER CODE BEGIN OnTxFrameCtrlChanged_2 */

	/* USER CODE END OnTxFrameCtrlChanged_2 */
}

static void OnPingSlotPeriodicityChanged(uint8_t pingSlotPeriodicity)
{
	/* USER CODE BEGIN OnPingSlotPeriodicityChanged_1 */

	/* USER CODE END OnPingSlotPeriodicityChanged_1 */
	LmHandlerParams.PingSlotPeriodicity = pingSlotPeriodicity;
	/* USER CODE BEGIN OnPingSlotPeriodicityChanged_2 */

	/* USER CODE END OnPingSlotPeriodicityChanged_2 */
}
#endif

#if MY_TX_OTAA
static void OnSystemReset(void)
{
	/* USER CODE BEGIN OnSystemReset_1 */

	/* USER CODE END OnSystemReset_1 */
	if ((LORAMAC_HANDLER_SUCCESS == LmHandlerHalt()) && (LmHandlerJoinStatus() == LORAMAC_HANDLER_SET))
	{
		NVIC_SystemReset();
	}
	/* USER CODE BEGIN OnSystemReset_Last */

	/* USER CODE END OnSystemReset_Last */
}
#endif

#if MY_NOUSE

static void StopJoin(void)
{
	/* USER CODE BEGIN StopJoin_1 */
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); /* LED_BLUE */
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET); /* LED_GREEN */
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET); /* LED_RED */
	/* USER CODE END StopJoin_1 */

	UTIL_TIMER_Stop(&TxTimer);

	if (LORAMAC_HANDLER_SUCCESS != LmHandlerStop())
	{
		APP_LOG(TS_OFF, VLEVEL_M, "LmHandler Stop on going ...\r\n");
	}
	else
	{
		APP_LOG(TS_OFF, VLEVEL_M, "LmHandler Stopped\r\n");
		if (LORAWAN_DEFAULT_ACTIVATION_TYPE == ACTIVATION_TYPE_ABP)
		{
			ActivationType = ACTIVATION_TYPE_OTAA;
			APP_LOG(TS_OFF, VLEVEL_M, "LmHandler switch to OTAA mode\r\n");
		}
		else
		{
			ActivationType = ACTIVATION_TYPE_ABP;
			APP_LOG(TS_OFF, VLEVEL_M, "LmHandler switch to ABP mode\r\n");
		}
		LmHandlerConfigure(&LmHandlerParams);
		LmHandlerJoin(ActivationType, true);
		UTIL_TIMER_Start(&TxTimer);
	}
	UTIL_TIMER_Start(&StopJoinTimer);
	/* USER CODE BEGIN StopJoin_Last */

	/* USER CODE END StopJoin_Last */
}

static void OnStopJoinTimerEvent(void *context)
{
	/* USER CODE BEGIN OnStopJoinTimerEvent_1 */

	/* USER CODE END OnStopJoinTimerEvent_1 */
	if (ActivationType == LORAWAN_DEFAULT_ACTIVATION_TYPE)
	{
		UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaStopJoinEvent), CFG_SEQ_Prio_0);
	}
	/* USER CODE BEGIN OnStopJoinTimerEvent_Last */
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET); /* LED_BLUE */
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET); /* LED_GREEN */
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET); /* LED_RED */
	/* USER CODE END OnStopJoinTimerEvent_Last */
}

static void StoreContext(void)
{
	LmHandlerErrorStatus_t status = LORAMAC_HANDLER_ERROR;

	/* USER CODE BEGIN StoreContext_1 */

	/* USER CODE END StoreContext_1 */
	status = LmHandlerNvmDataStore();

	if (status == LORAMAC_HANDLER_NVM_DATA_UP_TO_DATE)
	{
		APP_LOG(TS_OFF, VLEVEL_M, "NVM DATA UP TO DATE\r\n");
	}
	else if (status == LORAMAC_HANDLER_ERROR)
	{
		APP_LOG(TS_OFF, VLEVEL_M, "NVM DATA STORE FAILED\r\n");
	}
	/* USER CODE BEGIN StoreContext_Last */

	/* USER CODE END StoreContext_Last */
}

static void OnNvmDataChange(LmHandlerNvmContextStates_t state)
{
	/* USER CODE BEGIN OnNvmDataChange_1 */

	/* USER CODE END OnNvmDataChange_1 */
	if (state == LORAMAC_HANDLER_NVM_STORE)
	{
		APP_LOG(TS_OFF, VLEVEL_M, "NVM DATA STORED\r\n");
	}
	else
	{
		APP_LOG(TS_OFF, VLEVEL_M, "NVM DATA RESTORED\r\n");
	}
	/* USER CODE BEGIN OnNvmDataChange_Last */

	/* USER CODE END OnNvmDataChange_Last */
}

static void OnStoreContextRequest(void *nvm, uint32_t nvm_size)
{
	/* USER CODE BEGIN OnStoreContextRequest_1 */

	/* USER CODE END OnStoreContextRequest_1 */
	/* store nvm in flash */
	if (FLASH_IF_Erase(LORAWAN_NVM_BASE_ADDRESS, FLASH_PAGE_SIZE) == FLASH_IF_OK)
	{
		FLASH_IF_Write(LORAWAN_NVM_BASE_ADDRESS, (const void *)nvm, nvm_size);
	}
	/* USER CODE BEGIN OnStoreContextRequest_Last */

	/* USER CODE END OnStoreContextRequest_Last */
}

static void OnRestoreContextRequest(void *nvm, uint32_t nvm_size)
{
	/* USER CODE BEGIN OnRestoreContextRequest_1 */

	/* USER CODE END OnRestoreContextRequest_1 */
	FLASH_IF_Read(nvm, LORAWAN_NVM_BASE_ADDRESS, nvm_size);
	/* USER CODE BEGIN OnRestoreContextRequest_Last */

	/* USER CODE END OnRestoreContextRequest_Last */
}
#endif
