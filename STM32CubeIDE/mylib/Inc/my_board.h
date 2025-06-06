/*
 * my_board.h
 *
 *  Created on: Nov 12, 2024
 *      Author: roberto larosa
 */

#ifndef INC_MY_BOARD_SPECIFICS_H_
#define INC_MY_BOARD_SPECIFICS_H_

/*
 * Board Specific Definitions
 */

#define STEVAL_HARVEST1 1

#if STEVAL_HARVEST1

/* Sensor Integration section Begin */
#define MY_SoilSensor 0

#define MY_SHT40 1
#define SHT40_MEASURE_CMD_HP 0xFD 	// SHT40 Measure T & RH with highest precision (high repeatability)
#define SHT40_MEASURE_CMD_MP 0xF6 	// SHT40 Measure T & RH with medium precision (medium repeatability)
#define SHT40_MEASURE_CMD_LP 0xFD 	// SHT40 Measure T & RH with lowest precision (low repeatability)

#define MY_STTS22H 0
#define  STTS22H_CTRL_REG_VALUE 0X4C	// STTS22H Control Register: low_odr_start = 0, bdu = 1, avg = 00, if_add_inc = 1, freerun = 1, time_out_dis = 0, one_shot = 0

#define MY_STHS34PF80 0
#define MY_STHS34PF80_CONFIG 0

#define MY_LIS2DU12 0
// Control Register Default values for maximum performance
#define LIS2DU12_CTRL_REG1_VALUE 0x10	// wu_z_en = 0, wu_y_en = 0, wu_x_en = 0, drdy_pulsed = 0, if_add_inc = 1, sw_reset = 0, sim = 0, pp_od = 0
#define LIS2DU12_CTRL_REG2_VALUE 0x00	// int1_drdy = 0, int1_f_ovr = 0, int1_f_fth = 0, int1_f_full = 0, int1_boot = 0
#define LIS2DU12_CTRL_REG3_VALUE 0x00	// st = 0, int2_drdy = 0, int2_f_ovr = 0, int2_f_fth = 0, int2_f_full = 0, int2_boot = 0
#define LIS2DU12_CTRL_REG4_VALUE 0x10	// bdu = 1 (Block Data Update enabled), boot = 0, soc = 0, inact_odr = 0
#define LIS2DU12_CTRL_REG5_VALUE 0xB3	// fs = 0x03 (±16g full-scale), bw = 0x03 (400 Hz bandwidth), odr = 0x0B (800 Hz output data rate)
#define LIS2DU12_FIFO_CTRL_VALUE 0x00	// Default value (Bypass mode), f_mode = 0x00, stop_on_fth = 0, fifo_depth = 0, rounding_xyz = 0

/* Sensor Integration section End */

// Energy Storage (ES) GPIOs
#define ES_PortB GPIOB
#define ES_Pin1 GPIO_PIN_3 		// PB3 --> CN10[31]
#define ES_Pin2 GPIO_PIN_4		// PB4 --> CN7[34]
#define ES_Pin3 GPIO_PIN_14 	// PB14 --> CN7[36]

#define ES_PortC GPIOC
#define ES_Pin4 GPIO_PIN_13 	// PC13 --> CN7[23]


// Sensors GPIOs in I2C1
#define VDDS1_Port GPIOA
#define VDDS1_Pin GPIO_PIN_11	// PA11 --> CN10[5]

// Sensors GPIOs in I2C2
#define VDDS2_Port GPIOA
#define VDDS2_Pin GPIO_PIN_4	// PA4 --> CN10[17]

#endif

#endif /* INC_MY_BOARD_SPECIFICS_H_ */
