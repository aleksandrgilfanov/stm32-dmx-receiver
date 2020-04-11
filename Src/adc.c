#include <stdlib.h>
#include <string.h>

#include "adc.h"

static uint16_t* adc_value;
static size_t adc_buf_len;
static void (*adc_callback)(uint16_t *data);

volatile static uint8_t error;

bool adc_init(ADC_HandleTypeDef* hadc, uint8_t channels,
	      void (*cb_fn)(uint16_t *data))
{
	adc_callback = cb_fn;

	adc_buf_len = sizeof(*adc_value) * channels;

	adc_value = malloc(adc_buf_len);
	if (adc_value == NULL)
		return false;

	if (HAL_ADC_Start_DMA(hadc, (uint32_t*)adc_value, channels) != HAL_OK)
		return false;

	return true;
}

void adc_get(uint16_t *dst, uint8_t  *err)
{
	memcpy(dst, adc_value, adc_buf_len);
	*err = error;
}

/* Conversion complete for whole sequence of channels */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	adc_callback(adc_value);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc)
{
	error = 1;
}
