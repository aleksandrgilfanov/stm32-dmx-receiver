#ifndef CORE_H
#define CORE_H

#include <stdbool.h>

#define CORE_TIMER_CLOCK	 48000000
#define CORE_PWM_FREQUENCY	 1200
#define CORE_PWM_TIMER_PERIOD 	 999
#define CORE_PWM_TIMER_PRESCALER (-1 + CORE_TIMER_CLOCK / \
				 (CORE_PWM_FREQUENCY * (CORE_PWM_TIMER_PERIOD + 1)))

bool core_init(void);
void core_process(void);

#endif		/* CORE_H */
