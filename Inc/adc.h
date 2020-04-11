#ifndef ADC_H
#define ADC_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_adc.h"

bool adc_init(ADC_HandleTypeDef* hadc, uint8_t channels,
	      void (*cb_fn)(uint16_t *data));
void adc_get(uint16_t *dst, uint8_t  *err);

#endif		/* ADC_H */
