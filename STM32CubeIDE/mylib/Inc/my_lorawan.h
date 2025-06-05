/*
 * my_lorawan.h
 *
 *  Created on: Aug 9, 2024
 *      Author: Roberto La Rosa
 */

#ifndef MYLIB_INC_MY_LORAWAN_H_
#define MYLIB_INC_MY_LORAWAN_H_

#include "my_board.h"

// Private Includes
#include "my_sensor_app.h"
#include "main.h"
#include "utilities_def.h"
#include "stm32_seq.h"
#include "usart_if.h"
#include "gpio.h"
#include "se-identity.h"
#include "rtc.h"
#include "comp.h"
#include "timer_if.h"
#include "LoRaMac.h"
#include "app_x-cube-ai.h"
#if STEVAL_HARVEST1
#include "stm32wlxx_nucleo_radio.h"
#include "stm32wlxx_nucleo_bus.h"
#include "sht40ad1b.h"
#include "stts22h_reg.h"
#include "sths34pf80_reg.h"
#include "lis2du12.h"
#endif

// Private Defines

#define FLASH_DEVADDR_ADDRESS 0x1FFF7580

#define MY_DEBUGGER_ENABLED	0		// Enable Debug with the User Defined SystemInit
#define MY_SYSTEM_INIT	0			// 0 -> User Defined SystemInit; 1 -> Default SystemInit
#define MY_HARDWARE_DEBUGGER 0 		// Used to Perform Debug with GPIOs toggle

#define  MY_LPTIM_Monitor 0			// Used to monitor LPTIM Timer
#define  MY_RTCA_Monitor 0			// Used to monitor RTCA Timer
#define  MY_RTCB_Monitor 0			// Used to monitor RTCB Timer

#define FLASH_USER_END_ADDR     FLASH_END_ADDR								// End address of the last page of user flash memory
#define FLASH_USER_START_ADDR   (FLASH_USER_END_ADDR - FLASH_PAGE_SIZE) 	// Start address of the last page of user flash memory (0x0803F800)
#define FROM_FLASH 0														// If 1 the system uses data stored in the FLASH

#define MY_NOUSE 0
#define MY_PRINT 0
#define MY_LEDS 0					//  0 -> Nucleo Board LEDs are Disabled; 1 -> Board LEDs are Enabled
#define MY_TX 1						//  0 -> Radio is in Receiving mode; 1 -> Radio is in Transmitting mode

#define DIVR( x, y )         (((x)+((y)/2))/(y))
#define MY_RADIO_DEACTIVATION 0
#define MY_RTC_INIT 1 				// 	0 -> Default RTC Configuration; 1 -> User Defined RTC Configuration

/* Feature Definition Section Begin */
#define MY_UNDERVOLTING_FEATURE 1	// Configures Undervolting Feature
#define MY_ES_FEATURE_ENABLE 1 		// Enables Energy Storage (ES) Feature
#define MY_SM_FEATURE_ENABLE 1		// Enables Sensor Measurement (SM) Feature
#define MY_EMM_FEATURE_ENABLE 0 	// Enables EMS Measurement (DM) Feature
#define MY_EMT_FEATURE_ENABLE 0 	// Enables EMS Transmission (DT)Feature
/* Feature Definition Section End */

/* LoRA Radio Defines Section Begin */
#if MY_TX
#define MY_TX_ABP 1

#if MY_TX_ABP
#define LORAWAN_DEFAULT_ACTIVATION_TYPE		ACTIVATION_TYPE_ABP               	// LoRaWAN default activation type (OTAA, ABP)
#endif
#define AI	1

#if !MY_TX_ABP
#define MY_TX_OTAA 1
#define LORAWAN_DEFAULT_ACTIVATION_TYPE		ACTIVATION_TYPE_OTAA
#endif

#define ACTIVE_REGION						LORAMAC_REGION_EU868              	// LoraWAN application configuration REGIOIN EUROPE
#define APP_TX_DUTYCYCLE					10000                             	// Application data transmission duty cycle for default mode . 10s, value in [ms]
#define MY_RADIO_TX_IDLE_TIME				30000	                          	// Application data transmission duty cycle for not default mode in the power supply mode value in [ms]
#define LORAWAN_USER_APP_PORT				2								  	// LoRaWAN User application port, never use port 224 It is reserved for certification
#define LORAWAN_SWITCH_CLASS_PORT			3								  	// LoRaWAN User switch class port, never use port 224 It is reserved for certification
#define LORAWAN_DEFAULT_CLASS				CLASS_A							  	// LoRaWAN class mode
#define LORAWAN_DEFAULT_CONFIRMED_MSG_STATE	LORAMAC_HANDLER_UNCONFIRMED_MSG   	// LoRaWAN default confirm state
#define LORAWAN_ADR_STATE					LORAMAC_HANDLER_ADR_OFF           	// LoRaWAN Adaptive Data Rate, when ADR is enabled the end-device should be static
#define LORAWAN_DEFAULT_DATA_RATE			DR_1								// LORAWAN_DEFAULT_DATA_RATE is used only when LORAWAN_ADR_STATE is disabled (DR_0=SF12,DR_1=SF11,DR_2=SF10,DR_3=SF9,DR_4=SF8,DR_5=SF7)
#define LORAWAN_DEFAULT_TX_POWER			TX_POWER_6                        	// LORAWAN_DEFAULT_TX_POWER the end-device uses the XXXX_DEFAULT_TX_POWER value
#define LORAWAN_FORCE_REJOIN_AT_BOOT		false                             	// LoRaWAN force rejoin
#define LORAWAN_APP_DATA_BUFFER_MAX_SIZE	242								  	// User application data buffer size, 13 Bytes used for HEADER[MHDR(1)+FHDR(7)+MIC(4)]
#define DISABLE_LORAWAN_RX_WINDOW 			1                                	// Disable the ClassA receive windows after Tx
#define N_LORA_TX							1									// Define the number of Transmission to be done before swhitching to ES
#endif
/* LoRA Radio Defines Section End */

/*
Power consumption, All device parameters (Spreading Factor, channels selection, Tx Power, ...) should be fixed
and the adaptive datarate should be disabled.
In @file (se-identity.h) it is possible to provide DEV ADD, NETWORK SESSION KEY, APPLICATION SESSION KEY, AND DEV EUI
 */

/* Power Voltage Detector Levels Section Begin */

#define V2P0 PWR_PVDLEVEL_0		// ~ 2.0 Volt
#define V2P2 PWR_PVDLEVEL_1		// ~ 2.2 Volt
#define V2P4 PWR_PVDLEVEL_2		// ~ 2.4 Volt
#define V2P5 PWR_PVDLEVEL_3		// ~ 2.5 Volt
#define V2P6 PWR_PVDLEVEL_4		// ~ 2.6 Volt
#define V2P8 PWR_PVDLEVEL_5		// ~ 2.8 Volt
#define V2P9 PWR_PVDLEVEL_6		// ~ 2.9 Volt

#define VPS_CHECK V2P9          // Define Power Supply Threshold

#define VEOC V2P6				// Define End of Charge Voltage Threshold

#define VRTS V2P9 				// Define the Highest Voltage in RTS

#define VESS_HIGH V2P6			// Define the Highest Voltage in ESS
#define VESS_LOW V2P2			// Define the Lowest Voltage in ESS

#define VSMS_HIGH V2P9			// Define the Highest Voltage in SMS / DCM
#define VSMS_LOW V2P4			// Define the Lowest Voltage in SMS / DCM

#define VAIS_HIGH V2P9			// Define the Highest Voltage in AIS
#define VAIS_LOW V2P5			// Define the Lowest Voltage in AIS

#define PVD_EMS VEOC			//	PVD Threshold set in EMS it is equal to the first PVD level higher than the VEOC

/* Power Voltage Detector Levels Section End */

/* I2C Clock Speed Definition Begin */

//#define I2C_CLOCK_SPEED 0x20303E5D	// T = 42 usec
//#define I2C_CLOCK_SPEED 0x10805D88 // T = 68 usec
#define I2C_CLOCK_SPEED 0x9010DEFF // T = 641 usec
//#define I2C_CLOCK_SPEED 0XC0116EFF // T = 1 ms

/* I2C Clock Speed Definition End */

/* Private Variables Section Begin */
typedef enum {DATA_SIZE_8BIT, DATA_SIZE_16BIT, DATA_SIZE_32BIT} DataSize;

typedef enum {DCM, CCM, EOC} My_Cmode_td;
// DCM = Discontinuous Charge Mode
// CCM = Continuous Charge Mode
// EOC = End of Charge

typedef enum {MY_I2C1, MY_I2C2} My_I2C_td;

typedef enum {NVIC_ENABLE, NVIC_DISABLE, NVIC_CLEAR} My_Interrupts_Manager_td;

typedef enum {Switch_OPEN, Switch_CLOSED} My_Switch_td;

typedef enum {START_TIMER, STOP_TIMER} My_Timer_td;

typedef enum {CSS, PSS, RTS, EHS, ESS, EMS, EDS, SMS, RMS, AIS} My_Mode_State_td;
// CSS = Cold Start State : The System is powered by an Harvester.
// PSS = Power Supply State : The System is powered by a Power Supply with voltage higher the 3.0 V.
// RTS = Radio Transmission State : The System performs a Radio Transmission.
// EHS = Energy Harvesting State : The System is in Stop2 Power Mode to Harvest Energy.
// ESS = Energy Storage State : The System charges an Energy Storage Device (ESD), e.g., a Super Cap or a Battery.
// EMS = Energy Missing State: No energy to harvest is available for Energy Harvesting. The System is supplied by the Energy Storage Device.
// EDS = Energy Detection State: The System checks for availability of energy to harvest.
// SMS = Sensor Measurement State: The system activates the Sensor and performs measurements.
// RMS = Resistance Measurement State: The system Measures an external resistance.
// AIS = Artificial Intelligence State: The system runs the pre-trained model bases on the data gathered by the sensors

/* Private Variables Section End */

/* Timing Definition Section Begin */

/*
 * The Energy Storage Window (T_ESW) is a time window where Energy Harvesting State (EHS) and Energy Storage State (ESS) alternate continuously.
 */
#define T_ESW 5128	// Energy Storage Window (T_ESW), Time Window:	Time in seconds = T_ESW * 0.0039
#define DCM_MAX 5
#define SMS_MAX 5

/*
 * The Sensor Measurement Window (T_SMW) is a time window where Energy Harvesting State (EHS) and Sensor Measurement State (SMS) alternate continuously.
 */
#define T_SMW 5128	// Sensor Measurement Window (T_ESW), Time Window:	Time in seconds = T_SMW * 0.0039

/*
 * The T_EDS defines the maximum time that the EDS can last.
 * If the Energy Detection State (EDS) lasts less then T_EDS, the System switches  from EDS to EHS.
 * If the Energy Detection State (EDS) lasts longer than T_EDS the System switches from EDS to EDM.
 */
#define T_EDS 15000 // Energy Detection State (T_EDS), Time Window:	Time in seconds = T_EDS * 0.0039

/*
 * The T_EHS_EMS time defines the maximum time that the EHS can last.
 * If the EHS lasts less than T_EHS_EMS, the System switches from EHS to next defined state.
 * If the EHS lasts longer than T_EHS_EMS, the System switches from EHS to EMS.
 * NOTE: Write Number in Hexadecimal !!!// NOTE: Write Number in Hexadecimal !!!
 */
#define T_EHS_EMS_hours 0x0
#define T_EHS_EMS_minutes 0x4
#define T_EHS_EMS_seconds 0x0
#define T_EHS_EMS_subseconds 0x0

/*
 * The T_DCM_CCM time defines the maximum time the EHS can last while the system is in the Energy Storage Window (T_ESW).
 * If the EHS lasts less the T_DCM_CCM, the Syste, switches to next defined state.
 * If the EHS lasts longer than T_DCM_CCM, the System switches from Discontinuous Charge Mode (DCM) to Continuous Charge Mode (CCM) Charging Mode.
 * NOTE: Write Number in Hexadecimal !!!
 */
#define T_DCM_CCM_hours 0x0
#define T_DCM_CCM_minutes 0x0
#define T_DCM_CCM_seconds 0x7
#define T_DCM_CCM_subseconds 0x0

/*
 * IF the System is in the EMS, it periodically detects for the presence of Energy by switching into the EDS.
 * The time T_EMS_to_EDS, defines the period that the System detects the presence of Energy.
 * NOTE: Write Number in Hexadecimal !!!
 */
#define T_EMS_to_EDS_hours 0x0
#define T_EMS_to_EDS_minutes 0x2
#define T_EMS_to_EDS_seconds 0x0
#define T_EMS_to_EDS_subseconds 0x0

/*
 * IF the System is in the EMS, it periodically switches the SMS.
 * The time T_EMS_SMS, defines the period that the System switches into SMS.
 * NOTE: Write Number in Hexadecimal !!!
 */
#define T_EMS_SMS_hours 0x0
#define T_EMS_SMS_minutes 0x0
#define T_EMS_SMS_seconds 0x1E
#define T_EMS_SMS_subseconds 0x0

/* Delay used for the PVD to settle */
#define T_Delay_PVD_hours 0x0
#define T_Delay_PVD_minutes 0x0
#define T_Delay_PVD_seconds 0x2
#define T_Delay_PVD_subseconds 0x0

// Time To wait before declaring the Impedance Measurement Unsuccessful
#define T_LPTIM_SECOND 256 //LPTIM1 Clock cycles -> Elapsed Time = #Clock Cycles * 0.0039

// Dimension of the buffer to be filled during DARK operation
#define MY_DIM_EMS_BUFFER 9

//	PVD Threshold set in DARK Mode it is equal to the first PVD level higher than the VEOC
#define PVD_DARK VEOC

// Private Function prototypes
void My_PS_Detection(void);
void HAL_PWR_PVDCallback(void);

void My_RTC_Init(void);
void My_Interrupts_Manager(My_Interrupts_Manager_td My_Interrupts_Status);
void My_Enter_Stop2_Mode_WFI(uint32_t PWR_PVDLEVEL, uint32_t PWR_PVD_MODE_IT);
void My_Enter_Stop2_Mode_WFI_1(uint32_t PWR_PVDLEVEL, uint32_t PWR_PVD_MODE_IT);

void My_Exit_Stop2_Mode_WFI(void);

void My_Set_PVD(uint32_t PWR_PVDLEVEL, uint32_t PWR_PVD_MODE_IT);
void My_Set_PVD_WFI(uint32_t PWR_PVDLEVEL, uint32_t PWR_PVD_MODE_IT);
void My_PVD_Delay(void);

void My_MX_GPIO_Init(void);
void My_Adv_Data_Init(void);
void My_Set_GPIO_ANALOG(void);

void My_VDD_to_ES_Switch(My_Switch_td mode);
void My_VDD_to_VDDS1_Switch(My_Switch_td mode);
void My_VDD_to_VDDS2_Switch(My_Switch_td mode);
void My_LED_BLUE_Switch(My_Switch_td mode);

void My_Trise_Meas(void);

void My_EHF(void);
void My_ESF(void);
void My_RMF(void);
void My_SMF(void);
void My_AIF(void);

void My_Set_RTC_Alarm_A(uint8_t my_hours, uint8_t my_minutes, uint8_t my_seconds, uint32_t my_subseconds);
void My_Set_RTC_Alarm_B(uint8_t my_hours, uint8_t my_minutes, uint8_t my_seconds, uint32_t my_subseconds);

void My_ESW_DCM_Timer(My_Timer_td mode);
void My_ESW_CCM_Timer(My_Timer_td mode);
void My_SMW_Timer(My_Timer_td mode);
void My_EDS_Timer(My_Timer_td mode);
void My_EHS_to_EMS_Timer(My_Timer_td mode);
void My_DCM_to_CCM_Timer(My_Timer_td mode);
void My_EMS_to_EDS_Timer(My_Timer_td mode);
void My_EMS_to_SMS_Timer(My_Timer_td mode);
void My_SMS_Timer(My_Timer_td mode);

void My_SystemClock_Update(uint32_t CLOCK_RANGE);
void My_SystemClock_Config(void);
void My_GPIO_Out_Init(void);
void My_MX_LPTIM2_Init(void);
void My_Enable_HSI(void);
void My_Disable_HSI(void);

HAL_StatusTypeDef My_MX_I2C1_Init(I2C_HandleTypeDef* hi2c);
void My_I2C1_MspInit(I2C_HandleTypeDef* i2cHandle);
void My_I2C1_MspDeInit(I2C_HandleTypeDef* i2cHandle);

HAL_StatusTypeDef My_MX_I2C2_Init(I2C_HandleTypeDef* hi2c);
void My_I2C2_MspInit(I2C_HandleTypeDef* i2cHandle);
void My_I2C2_MspDeInit(I2C_HandleTypeDef* i2cHandle);

void My_I2C_Init(My_I2C_td mode);
void My_I2C_Deinit(My_I2C_td mode);

void My_HAL_Delay(__IO uint32_t Delay);

void My_SoilSensor_Get_Data(void);
void My_SHT40_Get_Data(void);
void My_STTS22H_Get_Data(void);
void My_STHS34PF80_Get_Data(void);
void My_LIS2DU12_Get_Data(void);

#if FROM_FLASH
void My_Write_Flash_Byte(uint32_t address, uint32_t data);
void My_Write_Flash_Page(void);
uint32_t My_Read_Flash_Data(uint32_t address, DataSize size);
void My_Read_Flash_Page(void);
#endif

void My_Set_All_GPIO_To_Analog_Mode(void);

uint32_t GetDeviceAddress(void);

void My_LPTIM1_Monitor(My_Switch_td mode);
void My_RTCA_Monitor(My_Switch_td mode);
void My_RTCB_Monitor(My_Switch_td mode);

//void My_DARK_Resistance_Measurement(void);

#endif /* MYLIB_INC_MY_LORAWAN_H_ */
