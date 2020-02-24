/*
 * main.h
 *
 *  Created on: 09.01.2020
 *      Author: andru
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "ch.h"

#include "modbus_slave.h"

#define FIRMWARE		102		// fw version
#define FLASHERASE      0       // if config size changed set first run to 1
#define DEBUG			0
#define	WDG				1
#define TIME_ON			30

typedef enum {
	EVT_TEST	= (1 << 0),
	EVT_IN		= (1 << 1),
	EVT_LED		= (1 << 2),
	EVT_MBCOIL	= (1 << 3),
	EVT_MBHOLD	= (1 << 4),
} EVT_MASK_t;

// eeprom configuration format
typedef struct CONFIG CONFIG_T;
struct CONFIG {
	uint16_t time1;				// light on channel 1 time, sec
	uint16_t time2;				// light on channel 2 time, sec
	uint8_t mb_addr;			// modbus address
	MB_BITRATE_MAP mb_bitrate;	// modbus bitrate enum
	MB_PARITY_MAP mb_parity;	// modbus parity enum
	bool en_local;				// local mode, no modbus control
	bool en_ch1;				// enable channel 1
	bool en_ch2;				// enable channel 2
	bool en_in;					// enable input sensor
	uint8_t light1;				// channel 1 default level
	uint8_t light2;				// channel 2 default level
};

// eeprom config length
#define CFGLEN		( sizeof(CONFIG_T) )

#define THD_GOOD	0b1111
#define THD_INIT	0

typedef enum {
	THD_MAIN	= (1 << 0),
	THD_EVENT	= (1 << 1),
	THD_MODBUS  = (1 << 2),
	THD_SWITCH	= (1 << 3),
} thd_check_t;

extern volatile thd_check_t thd_state;
extern event_source_t event_src;
extern CONFIG_T config;

#endif /* MAIN_H_ */
