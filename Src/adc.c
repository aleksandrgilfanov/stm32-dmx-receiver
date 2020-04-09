#include "adc.h"

volatile static uint32_t adc_value;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	adc_value = HAL_ADC_GetValue(hadc);

	HAL_ADC_Start_IT(hadc);
}

void adc_init(ADC_HandleTypeDef* hadc)
{
	HAL_ADC_Start_IT(hadc);
}
