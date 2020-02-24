/*
 * led.h
 *
 *  Created on: 09/01/2019
 *      Author: andru
 */

#ifndef LED_H_
#define LED_H_

#define LEDCH		2	// PWM channels
#define LEDCH1		0
#define LEDCH2		1

#define LEDPWM		PWMD2
#define PWMCH1		0
#define PWMCH2		1

#define PWM_BASE 	255
#define PWM_PERIOD	1000
#define PWM_FREQ	100000

#define CH_STEP		1
#define CH_SPEED	50
#define CH_UPSTART	10

typedef enum {
	CH_OFF = 0,
	CH_ON,
	CH_STEP_UP,
	CH_STEP_DOWN,
} ch_mode_t;

typedef struct _channel_t channel_t;
struct _channel_t {
	uint8_t   channel;
	ch_mode_t mode;
	uint8_t   level;
};

typedef struct _dimmer_t dimmer_t;
struct _dimmer_t {
	uint8_t   target;
	uint8_t   delta;
	uint16_t  speed;
	uint16_t  counter;
};

typedef struct _led_t led_t;
struct _led_t {
	channel_t channel[LEDCH];
};

#include "main.h"

bool set_channel(uint8_t ch, channel_t *config);
bool set_dimmer(uint8_t ch, const dimmer_t *config);
void led_init(void);

extern led_t led;

#endif /* LED_H_ */
