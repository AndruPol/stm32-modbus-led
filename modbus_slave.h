/*
 * modbus_slave.h
 *
 *  Created on: 29 июня 2015 г.
 *      Author: andru
 */

#ifndef MODBUS_SLAVE_H_
#define MODBUS_SLAVE_H_

#include "port.h"

/* -----------------------Slave Defines ------------------------------------- */
#define S_DISCRETE_INPUT_START        0
#define S_DISCRETE_INPUT_NDISCRETES   8
#define S_COIL_START                  0
#define S_COIL_NCOILS                 14
#define S_REG_INPUT_START             0
#define S_REG_INPUT_NREGS             9
#define S_REG_HOLDING_START           0
#define S_REG_HOLDING_NREGS           8
/* slave mode: holding register's all address */
#define S_HD_RESERVE                  0
#define S_HD_CPU_USAGE_MAJOR          1
#define S_HD_CPU_USAGE_MINOR          2
/* salve mode: input register's all address */
#define S_IN_RESERVE                  0
/* salve mode: coil's all address */
#define S_CO_RESERVE                  0
/* salve mode: discrete's all address */
#define S_DI_RESERVE                  0

//Slave mode:DiscreteInputs variables
extern USHORT   usSDiscInStart;
extern UCHAR    ucSDiscInBuf[];
//Slave mode:Coils variables
extern USHORT   usSCoilStart;
extern UCHAR    ucSCoilBuf[];
//Slave mode:InputRegister variables
extern USHORT   usSRegInStart;
extern SHORT   usSRegInBuf[];
//Slave mode:HoldingRegister variables
extern USHORT   usSRegHoldStart;
extern SHORT   usSRegHoldBuf[];

#define MB_ADDRESS	32
#define MB_BITRATE	MB_BITRATE_19200
#define MB_PARITY	MB_PARITY_NONE

typedef enum {
	OFF = 0,
	ON,
} MB_OFFON_T;

typedef enum {
	MB_DI_CH1,			// channel 1 on/off
	MB_DI_EN_CH1,		// channel 1 enable
	MB_DI_CH2,			// channel 2 on/off
	MB_DI_EN_CH2,		// channel 2 enable
	MB_DI_IN,			// move sensor state
	MB_DI_EN_IN,		// move sensor enable
	MB_DI_FLASH_ERR,	// eeprom read error
	MB_DI_TEMP_ERR,		// temperature read error
} MB_DISCRETE_MAP;

typedef enum {
	MB_CO_LOCAL,	// local mode
	MB_CO_EN_IN,	// enable input sensor
	MB_CO_CH1,		// set channel 1
	MB_CO_EN_CH1,	// enable channel 1
	MB_CO_LIGHT1,	// set default light level on channel 1
	MB_CO_TIME1,	// set local mode time on channel 1, sec
	MB_CO_SOFT1,	// soft mode on channel 1
	MB_CO_CH2,		// set channel 2
	MB_CO_EN_CH2,	// enable channel 2
	MB_CO_LIGHT2,	// set default light level on channel 2
	MB_CO_TIME2,	// set local mode time on channel 2, sec
	MB_CO_SOFT2,	// soft mode on channel 2
	MB_CO_MBADDR,	// set modbus address flag
	MB_CO_MBCONF,	// set modbus parameters flag
} MB_COIL_MAP;

typedef enum {
	MB_IN_FIRMWARE,		// firmware version
	MB_IN_COUNTER,		// dummy counter
	MB_IN_TEMPERATURE,	// stm32 temperature
	MB_IN_LIGHT1,		// default light on channel 1
	MB_IN_TIME1,		// local mode on channel 1 time, sec
	MB_IN_LIGHT2,		// default light on channel 2
	MB_IN_TIME2,		// local mode on channel 2 time, sec
	MB_IN_TIMER1,		// timer1 current value
	MB_IN_TIMER2,		// timer2 current value
} MB_INPUT_MAP;

typedef enum {
	MB_HO_MBADDR,	// modbus address
	MB_HO_MBCONF,	// bitrate & parity
	MB_HO_LIGHT1,	// light level on channel 1, 0-255
	MB_HO_TIME1,	// set timer on channel 1, sec
	MB_HO_LIGHT2,	// light level on channel 2, 0-255
	MB_HO_TIME2,	// set timer on channel 2, sec
	MB_HO_SOFT_UP,	// soft up time, sec
	MB_HO_SOFT_DOWN,// soft down time, sec
} MB_HOLDING_MAP;

typedef enum {
	MB_BITRATE_1200	= 1,
	MB_BITRATE_2400,
	MB_BITRATE_4800,
	MB_BITRATE_9600,
	MB_BITRATE_19200,
	MB_BITRATE_38400,
	MB_BITRATE_57600,
	MB_BITRATE_115200,
} MB_BITRATE_MAP;

typedef enum {
	MB_PARITY_NONE = 0,
	MB_PARITY_EVEN,
	MB_PARITY_ODD,
} MB_PARITY_MAP;

// create & start modbus poll process
void modbus_init(void);
void setDiscBit(uint16_t regAddr, uint8_t ucValue);
uint8_t getCoilBit(uint16_t regAddr);
void setCoilBit(uint16_t regAddr, uint8_t ucValue);
void writeInputReg(uint16_t regAddr, uint16_t regValue);
USHORT readHoldingReg(uint16_t regAddr);
void writeHoldingReg(uint16_t regAddr, USHORT regValue);

#endif /* MODBUS_SLAVE_H_ */
