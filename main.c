/*
 * main.c
 *
 * 2 channel MODBUS dimmer
 *
 *  Created on: 09.01.2020
 *      Author: andru
 */

#include <string.h>

#include "ch.h"
#include "hal.h"

#include "main.h"
#include "crc8.h"
#include "modbus_slave.h"
#include "sensors.h"
#include "hal_flash_lld.h"
#include "stm32_flash.h"
#include "led.h"

#define LineRead(l)     palReadLine(l)
#define LineSet(l, n)   ((n) ? palSetLine(l) : palClearLine(l))
#define LED				PAL_LINE(GPIOB, GPIOB_LED)
#define IN      		PAL_LINE(GPIOC, GPIOC_IN)
#define SENSOR_POLL		250

volatile thd_check_t thd_state;		// thread state mask
CONFIG_T config;					// configuration data
event_source_t event_src;			// event thread event sources

volatile uint8_t dim1, dim2, in;
volatile uint16_t timer1, timer2;
static binary_semaphore_t cfgsem;

void halt(void);
static void modbus_coil(void);
static void modbus_hold(void);

#if WDG
/*
 * Watchdog deadline set to 2000 / (LSI=40000 / 64) = ~3.2s.
 */
static const WDGConfig wdgcfg = {
  STM32_IWDG_PR_64,
  STM32_IWDG_RL(2000),
};
#endif

/*===========================================================================*/
/* Main and generic code.                                                    */
/*===========================================================================*/

static THD_WORKING_AREA(waCfgThread, 512);
static THD_FUNCTION(cfgThread, arg) {
    (void)arg;

	chBSemObjectInit(&cfgsem, TRUE);
	chRegSetThreadName("cfg");

	while (true) {
		chBSemWait(&cfgsem);
		setDiscBit(MB_DI_FLASH_ERR, OFF);
#if WDG
    	wdgReset(&WDGD1);
#endif
		if (!writeFlash((uint8_t *) &config))
			  setDiscBit(MB_DI_FLASH_ERR, ON);
	}
}

static THD_WORKING_AREA(waSwitchThread, 256);
static THD_FUNCTION(switchThread, arg) {
        (void)arg;

        chRegSetThreadName("switch");
        while (true) {
                uint8_t state;

                thd_state |= THD_SWITCH;

                if (config.en_in) {
                	state = LineRead(IN);
                    setDiscBit(MB_DI_IN, state);
                	if (state)
                		chEvtBroadcastFlags(&event_src, EVT_IN);
                }
                chThdSleepMilliseconds(SENSOR_POLL);

        }
}

static THD_WORKING_AREA(waEventThread, 512);
static THD_FUNCTION(eventThread, arg) {
	(void)arg;

	event_listener_t event_el;

	chRegSetThreadName("Event");
	chEvtObjectInit(&event_src);
	chEvtRegisterMask(&event_src, &event_el, ALL_EVENTS);

	while (true) {
		chEvtWaitAny(ALL_EVENTS);
		eventflags_t flags = chEvtGetAndClearFlags(&event_el);

		if (flags & EVT_TEST)
		    thd_state |= THD_EVENT;

		if (flags & EVT_IN && config.en_local) {
			channel_t chcfg;
			if (config.en_ch1) {
				chcfg.level = config.light1;
				chcfg.mode = CH_ON;
				set_channel(LEDCH1, &chcfg);
				timer1 = config.time1;
			}
			if (config.en_ch2) {
				chcfg.level = config.light2;
				chcfg.mode = CH_ON;
				set_channel(LEDCH2, &chcfg);
				timer2 = config.time2;
			}
		}

	    // Modbus coil write event
	    if (flags & EVT_MBCOIL) {
	    	modbus_coil();
	    } //EVT_MBCOIL

	    // Modbus hold write event
	    if (flags & EVT_MBHOLD) {
	    	modbus_hold();
	    } //EVT_MBHOLD

	    if (flags & EVT_LED) {
			setDiscBit(MB_DI_CH1, led.channel[LEDCH1].mode != CH_OFF);
			setDiscBit(MB_DI_CH2, led.channel[LEDCH2].mode != CH_OFF);
			setCoilBit(MB_CO_CH1, led.channel[LEDCH1].mode != CH_OFF);
			setCoilBit(MB_CO_CH2, led.channel[LEDCH2].mode != CH_OFF);
		}
	} //while
}

static void default_config(void) {
  config.mb_addr = MB_ADDRESS;
  config.mb_bitrate = MB_BITRATE;
  config.mb_parity = MB_PARITY_NONE;
  config.en_ch1 = config.en_ch2 = config.en_in = true;
  config.en_local = true;
  config.light1 = config.light2 = PWM_BASE;
  config.time1 = config.time2 = TIME_ON;
}

static void modbus_coil(void) {
	uint8_t bit, write = false;
	uint16_t val;
	channel_t chcfg;
	dimmer_t dimcfg;

	if (config.en_local)
		goto config_coil;

	bit = getCoilBit(MB_CO_CH1) != 0;
	if (!bit && led.channel[LEDCH1].mode > CH_OFF) {
		chcfg.level = 0;
		chcfg.mode = CH_OFF;
		set_channel(LEDCH1, &chcfg);
	}
	if (bit && led.channel[LEDCH1].mode == CH_OFF) {
		val = readHoldingReg(MB_HO_LIGHT1);
		chcfg.level = config.light1;
		if (val > 0 && val <= PWM_BASE) {
			chcfg.level = val;
		}
		// soft up?
		bit = getCoilBit(MB_CO_SOFT1) != 0;
		val = readHoldingReg(MB_HO_SOFT_UP);
		if (bit && val > 0) {
			dimcfg.target = chcfg.level;
			chcfg.mode = CH_STEP_UP;
			chcfg.level = CH_UPSTART;
			dimcfg.delta = CH_STEP;
			dimcfg.speed = val * PWM_PERIOD / ((dimcfg.target - chcfg.level - led.channel[LEDCH1].level) / CH_STEP) + 1;
			set_channel(LEDCH1, &chcfg);
			set_dimmer(LEDCH1, &dimcfg);
		} else {
			chcfg.mode = CH_ON;
			set_channel(LEDCH1, &chcfg);
		}
		timer1 = readHoldingReg(MB_HO_TIME1);
	}

	bit = getCoilBit(MB_CO_CH2) != 0;
	if (!bit && led.channel[LEDCH2].mode > CH_OFF) {
		chcfg.level = 0;
		chcfg.mode = CH_OFF;
		set_channel(LEDCH2, &chcfg);
	}
	if (bit && led.channel[LEDCH2].mode == CH_OFF) {
		val = readHoldingReg(MB_HO_LIGHT2);
		chcfg.level = config.light2;
		if (val > 0 && val <= PWM_BASE) {
			chcfg.level = val;
		}
		// soft up?
		bit = getCoilBit(MB_CO_SOFT2) != 0;
		val = readHoldingReg(MB_HO_SOFT_UP);
		if (bit && val > 0) {
			dimcfg.target = chcfg.level;
			chcfg.mode = CH_STEP_UP;
			chcfg.level = CH_UPSTART;
			dimcfg.delta = CH_STEP;
			dimcfg.speed = val * PWM_PERIOD / ((dimcfg.target - chcfg.level - led.channel[LEDCH2].level) / CH_STEP) + 1;
			set_channel(LEDCH2, &chcfg);
			set_dimmer(LEDCH2, &dimcfg);
		} else {
			chcfg.mode = CH_ON;
			set_channel(LEDCH2, &chcfg);
		}
		timer2 = readHoldingReg(MB_HO_TIME2);
	}

config_coil:

	if (getCoilBit(MB_CO_MBADDR)) {
		setCoilBit(MB_CO_MBADDR, OFF);
		uint16_t reg = readHoldingReg(MB_HO_MBADDR);
		if ((reg != config.mb_addr) && (reg > 0) && (reg < 247)) {
			config.mb_addr = reg;
			write = true;
		}
	}

	if (getCoilBit(MB_CO_MBCONF)) {
		setCoilBit(MB_CO_MBCONF, OFF);
		uint16_t reg = readHoldingReg(MB_HO_MBCONF);
		uint8_t bitrate = reg & 0xFF;
		uint8_t parity = reg >> 8;
		if ((bitrate != config.mb_bitrate) && (bitrate >= MB_BITRATE_1200) && (bitrate <= MB_BITRATE_115200)) {
			config.mb_bitrate = bitrate;
			write = true;
		}
		if ((parity != config.mb_parity) && (parity <= MB_PARITY_ODD)) {
			config.mb_parity = parity;
			write = true;
		}
	}

	bit = getCoilBit(MB_CO_LIGHT1);
	if (bit) {
		setCoilBit(MB_CO_LIGHT1, OFF);
		val = readHoldingReg(MB_HO_LIGHT1);
		if (val <= 255 && config.light1 != val) {
			config.light1 = val;
			writeInputReg(MB_IN_LIGHT1, config.light1);
			write = true;
		}
	}

	bit = getCoilBit(MB_CO_LIGHT2);
	if (bit) {
		setCoilBit(MB_CO_LIGHT2, OFF);
		val = readHoldingReg(MB_HO_LIGHT2);
		if (val <= 255 && config.light2 != val) {
			config.light2 = val;
			writeInputReg(MB_IN_LIGHT2, config.light2);
			write = true;
		}
	}

	bit = getCoilBit(MB_CO_TIME1);
	if (bit) {
		setCoilBit(MB_CO_TIME1, OFF);
		val = readHoldingReg(MB_HO_TIME1);
		if (val > 0 && config.time1 != val) {
			config.time1 = val;
			writeInputReg(MB_IN_TIME1, config.time1);
			write = true;
		}
	}

	bit = getCoilBit(MB_CO_TIME2);
	if (bit) {
		setCoilBit(MB_CO_TIME2, OFF);
		val = readHoldingReg(MB_HO_TIME2);
		if (val > 0 && config.time2 != val) {
			config.time2 = val;
			writeInputReg(MB_IN_TIME2, config.time2);
			write = true;
		}
	}

	bit = getCoilBit(MB_CO_LOCAL);
	if (config.en_local != bit) {
		chcfg.level = 0;
		chcfg.mode = CH_OFF;
		set_channel(LEDCH1, &chcfg);
		set_channel(LEDCH2, &chcfg);
		timer1 = timer2 = 0;
		config.en_local = bit;
		write = true;
	}

	bit = getCoilBit(MB_CO_EN_IN);
	if (config.en_in != bit) {
		config.en_in = bit;
		write = true;
	}

	bit = getCoilBit(MB_CO_EN_CH1);
	if (config.en_ch1 != bit) {
		config.en_ch1 = bit;
		write = true;
	}

	bit = getCoilBit(MB_CO_EN_CH2);
	if (config.en_ch2 != bit) {
		config.en_ch2 = bit;
		write = true;
	}

	if (write) chBSemSignal(&cfgsem);
}

static void modbus_hold(void) {
	uint16_t val;
	channel_t chcfg;

	val = readHoldingReg(MB_HO_TIME1);
	if (val && led.channel[LEDCH1].mode > CH_OFF) {
		timer1 = val;
		val = readHoldingReg(MB_HO_LIGHT1);
		chcfg.level = config.light1;
		if (val > 0 && val <= PWM_BASE) {
			chcfg.level = val;
		}
		if (led.channel[LEDCH1].level != chcfg.level) {
			chcfg.mode = CH_ON;
			set_channel(LEDCH1, &chcfg);
		}
	}

	val = readHoldingReg(MB_HO_TIME2);
	if (val && led.channel[LEDCH2].mode > CH_OFF) {
		timer2 = val;
		val = readHoldingReg(MB_HO_LIGHT2);
		chcfg.level = config.light2;
		if (val > 0 && val <= PWM_BASE) {
			chcfg.level = val;
		}
		if (led.channel[LEDCH2].level != chcfg.level) {
			chcfg.mode = CH_ON;
			set_channel(LEDCH2, &chcfg);
		}
	}
}

// called on kernel panic
void halt(void) {
  port_disable();
  while (true)	{
	palToggleLine(LED);
	chThdSleepMilliseconds(250);
  }
}

/*
 * Application entry point.
 */
int main(void) {
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  palSetLineMode(LED, PAL_MODE_OUTPUT_PUSHPULL);
  palSetLineMode(IN, PAL_MODE_INPUT);

#if WDG
  wdgStart(&WDGD1, &wdgcfg);
#endif

  if (!initFlashInfo()) {
	  halt();
  }

  // firmware & config size changed
#if FLASHERASE
  if (!flashErase()) {
          halt();
  }
  if (!initFlashInfo()) {
          halt();
  }
#endif

  // read config from FLASH
  if (!readFlash((uint8_t *) &config)) {
	  setDiscBit(MB_DI_FLASH_ERR, ON);
	  default_config();
	  if (!writeFlash((uint8_t *) &config)) {
		  halt();
	  }
  }

  // Creates the MODBUS thread.
  modbus_init();

  writeInputReg(MB_IN_FIRMWARE, FIRMWARE);

  setDiscBit(MB_DI_EN_CH1, config.en_ch1);
  setDiscBit(MB_DI_EN_CH2, config.en_ch2);
  setDiscBit(MB_DI_EN_IN, config.en_in);
  setCoilBit(MB_CO_EN_IN, config.en_in);
  setCoilBit(MB_CO_LOCAL, config.en_local);
  setDiscBit(MB_DI_CH1, led.channel[LEDCH1].mode != CH_OFF);
  setCoilBit(MB_CO_CH1, led.channel[LEDCH1].mode != CH_OFF);
  setCoilBit(MB_CO_EN_CH1, config.en_ch1);
  setDiscBit(MB_DI_CH2, led.channel[LEDCH2].mode != CH_OFF);
  setCoilBit(MB_CO_CH2, led.channel[LEDCH2].mode != CH_OFF);
  setCoilBit(MB_CO_EN_CH2, config.en_ch2);

  writeInputReg(MB_IN_LIGHT1, config.light1);
  writeInputReg(MB_IN_LIGHT2, config.light2);
  writeInputReg(MB_IN_TIME1, config.time1);
  writeInputReg(MB_IN_TIME2, config.time2);

  writeHoldingReg(MB_HO_MBADDR, config.mb_addr);
  uint16_t reg = ((uint16_t) config.mb_parity << 8) | config.mb_bitrate;
  writeHoldingReg(MB_HO_MBCONF, reg);

  timer1 = timer2 = 0;
  writeHoldingReg(MB_HO_LIGHT1, config.light1);
  writeHoldingReg(MB_HO_LIGHT2, config.light2);
  writeHoldingReg(MB_HO_TIME1, timer1);
  writeHoldingReg(MB_HO_TIME2, timer2);

  led_init();

  // write to flash thread
  chThdCreateStatic(waCfgThread, sizeof(waCfgThread), NORMALPRIO+1, cfgThread, NULL);

  // event thread
  chThdCreateStatic(waEventThread, sizeof(waEventThread), NORMALPRIO+2, eventThread, NULL);

  // sensor thread
  chThdCreateStatic(waSwitchThread, sizeof(waSwitchThread), NORMALPRIO, switchThread, NULL);

  // ADC read thread
  sensors_init();

  uint16_t dummy=0;
  systime_t time = chVTGetSystemTime();
  while (TRUE) {
	time += TIME_MS2I(1000);

    palToggleLine(LED);
	chEvtBroadcastFlags(&event_src, EVT_TEST);
	writeInputReg(MB_IN_COUNTER, dummy++);

    thd_state |= THD_MAIN;
    if (thd_state == THD_GOOD) {
#if WDG
    	wdgReset(&WDGD1);
#endif
    }

    if (timer1 > 0 && led.channel[LEDCH1].mode > CH_OFF) {
		uint8_t bit = getCoilBit(MB_CO_SOFT1) != 0;
		uint16_t val = readHoldingReg(MB_HO_SOFT_DOWN);
    	timer1--;
		if (bit && val > 0 && val == timer1) {
			dimmer_t dimcfg;
			led.channel[LEDCH1].mode = CH_STEP_DOWN;
			dimcfg.target = 0;
			dimcfg.delta = CH_STEP;
			dimcfg.speed = val * PWM_PERIOD / (led.channel[LEDCH1].level / CH_STEP) + 1;
			set_dimmer(LEDCH1, &dimcfg);
		}
		if (timer1 == 0) {
			channel_t chcfg;
			chcfg.mode = CH_OFF;
			chcfg.level = 0;
			set_channel(LEDCH1, &chcfg);
		}
    }

    if (timer2 > 0 && led.channel[LEDCH2].mode > CH_OFF) {
		uint8_t bit = getCoilBit(MB_CO_SOFT2) != 0;
		uint16_t val = readHoldingReg(MB_HO_SOFT_DOWN);
    	timer2--;
		if (bit && val > 0 && val == timer2) {
			dimmer_t dimcfg;
			led.channel[LEDCH2].mode = CH_STEP_DOWN;
			dimcfg.target = 0;
			dimcfg.delta = CH_STEP;
			dimcfg.speed = val * PWM_PERIOD / (led.channel[LEDCH2].level / CH_STEP) + 1;
			set_dimmer(LEDCH2, &dimcfg);
		}
		if (timer2 == 0) {
			channel_t chcfg;
			chcfg.mode = CH_OFF;
			chcfg.level = 0;
			set_channel(LEDCH2, &chcfg);
		}
    }
    writeInputReg(MB_IN_TIMER1, timer1);
    writeInputReg(MB_IN_TIMER2, timer2);

    // ADC read
    if (sensors_read() == ADC_NO_ERROR) {
    	// mcu internal temperature calculation
    	float temp = TEMPCALC(adc_tempint, adc_vrefint);
    	writeInputReg(MB_IN_TEMPERATURE, 10 * temp);
		setDiscBit(MB_DI_TEMP_ERR, OFF);
    } else {
    	setDiscBit(MB_DI_TEMP_ERR, ON);
    }

    chThdSleepUntil(time);
  }
}
