#ifndef ADC_H
#define ADC_H

#include <stdint.h>

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_adc.h"

void adc_init(ADC_HandleTypeDef* hadc);

#endif		/* ADC_H */
