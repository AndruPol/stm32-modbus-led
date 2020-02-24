/*
 * led.c
 *
 *  Created on: 09/01/2019
 *      Author: andru
 */

#include "ch.h"
#include "hal.h"

#include "main.h"
#include "led.h"

static binary_semaphore_t dimsem;
static dimmer_t dimmer[LEDCH];
led_t led;

static void pwm_cb(PWMDriver *pwmp) {
  (void)pwmp;
  bool fired = false;

  for (uint8_t i=0; i<LEDCH; i++) {
	  if (led.channel[i].mode > CH_ON && dimmer[i].counter > 0) {
		  if (--dimmer[i].counter == 0) {
			  dimmer[i].counter = dimmer[i].speed;
			  fired = true;
		  }
	  }
  }

  if (fired) {
	 chSysLockFromISR();
	 chBSemSignalI(&dimsem);
	 chSysUnlockFromISR();
  }
}

static PWMConfig pwmcfg = {
  PWM_FREQ,
  (PWM_FREQ / PWM_PERIOD),
  pwm_cb,
  {
	{ PWM_OUTPUT_ACTIVE_LOW, NULL, },
    { PWM_OUTPUT_ACTIVE_LOW, NULL, },
    { PWM_OUTPUT_DISABLED,   NULL, },
    { PWM_OUTPUT_DISABLED,   NULL, },
  },
  0,
  0,
#if STM32_PWM_USE_ADVANCED
  0,
#endif
};

bool set_channel(uint8_t ch, channel_t *config) {
	if (ch >= LEDCH || config->mode > CH_STEP_DOWN)
		return false;

	led.channel[ch].mode = config->mode;
	led.channel[ch].level = config->level;

	switch (led.channel[ch].mode) {
	case CH_OFF:
		dimmer[ch].counter = 0;
		pwmDisableChannel(&LEDPWM, led.channel[ch].channel);
		break;
	case CH_ON:
		dimmer[ch].counter = 0;
		pwmEnableChannel(&LEDPWM, led.channel[ch].channel,
			PWM_FRACTION_TO_WIDTH(&LEDPWM, PWM_BASE, led.channel[ch].level));
		break;
	default:
		pwmEnableChannel(&LEDPWM, led.channel[ch].channel,
			PWM_FRACTION_TO_WIDTH(&LEDPWM, PWM_BASE, led.channel[ch].level));
		break;
	}
	chEvtBroadcastFlags(&event_src, EVT_LED);
	return true;
}

bool set_dimmer(uint8_t ch, const dimmer_t *config) {
	if (ch >= LEDCH || led.channel[ch].mode <= CH_ON)
		return false;
	if (led.channel[ch].mode == CH_STEP_UP &&
		(led.channel[ch].level >= config->target))
		return false;
	if (led.channel[ch].mode == CH_STEP_DOWN &&
		(led.channel[ch].level < config->target))
		return false;

	if (config->delta)
		dimmer[ch].delta = config->delta;
	else
		dimmer[ch].delta = CH_STEP;
	if (config->speed)
		dimmer[ch].speed = config->speed;
	else
		dimmer[ch].speed = CH_SPEED;
	dimmer[ch].target = config->target;
	dimmer[ch].counter = dimmer[ch].speed;

	return true;
}

static THD_WORKING_AREA(waDIMThd, 256);
static THD_FUNCTION(DIMThd, arg) {
    (void)arg;

    chRegSetThreadName("dimmer");
    chBSemObjectInit(&dimsem, TRUE);

    while (!chThdShouldTerminateX()) {
    	chBSemWait(&dimsem);
    	for (uint8_t i=0; i<LEDCH; i++) {
    		switch (led.channel[i].mode) {
    		case CH_STEP_UP:
        		if (led.channel[i].level < PWM_BASE && led.channel[i].level < dimmer[i].target) {
        			led.channel[i].level += dimmer[i].delta;
        			pwmEnableChannel(&LEDPWM, led.channel[i].channel,
        				PWM_FRACTION_TO_WIDTH(&LEDPWM, PWM_BASE, led.channel[i].level));
        		} else {
        			led.channel[i].mode = CH_ON;
        			chEvtBroadcastFlags(&event_src, EVT_LED);
        		}
    			break;
    		case CH_STEP_DOWN:
        		if (led.channel[i].level > 0 && led.channel[i].level > dimmer[i].target) {
        			led.channel[i].level -= dimmer[i].delta;
        			pwmEnableChannel(&LEDPWM, led.channel[i].channel,
        				PWM_FRACTION_TO_WIDTH(&LEDPWM, PWM_BASE, led.channel[i].level));
        		} else {
        			led.channel[i].mode = CH_OFF;
        			pwmDisableChannel(&LEDPWM, led.channel[i].channel);
        			chEvtBroadcastFlags(&event_src, EVT_LED);
        		}
    			break;
    		default:
    			break;
    		}
    	}
    } // while
    chThdExit((msg_t) 0);
}

void led_init(void) {
    for (uint8_t i=0; i < LEDCH; i++) {
    	led.channel[i].mode = CH_OFF;
    	led.channel[i].level = 0;
    	dimmer[i].delta = CH_STEP;
    	dimmer[i].speed = CH_SPEED;
    }
    led.channel[LEDCH1].channel = PWMCH1;
    led.channel[LEDCH2].channel = PWMCH2;

	pwmStart(&LEDPWM, &pwmcfg);
    pwmDisableChannel(&LEDPWM, led.channel[LEDCH1].channel);
    pwmDisableChannel(&LEDPWM, led.channel[LEDCH2].channel);
    pwmEnablePeriodicNotification(&LEDPWM);

    (void) chThdCreateStatic(waDIMThd, sizeof(waDIMThd), NORMALPRIO, DIMThd, NULL);
}

