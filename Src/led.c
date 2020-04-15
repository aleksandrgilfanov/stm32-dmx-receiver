#include "led.h"

static TIM_HandleTypeDef *ledtim;

void led_set(uint8_t led, uint16_t value)
{
	TIM_OC_InitTypeDef sConfigOC;

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = value;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	if (led == 0) {
		HAL_TIM_PWM_ConfigChannel(ledtim, &sConfigOC, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(ledtim, TIM_CHANNEL_1);
	}
	if (led == 1) {
		HAL_TIM_PWM_ConfigChannel(ledtim, &sConfigOC, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(ledtim, TIM_CHANNEL_2);
	}
}

bool led_init(TIM_HandleTypeDef *htim)
{
	TIM_OC_InitTypeDef sConfigOC;

	ledtim = htim;

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 666;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_2);

	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);

	return true;
}
