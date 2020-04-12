#include <stdint.h>

#include "main.h"

#include "core.h"
#include "adc.h"
#include "dmx_receiver.h"
#include "led.h"
#include "usb_debug.h"

#define ADC_CHANNELS 		2
#define ADC_MAX 		1800

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim4;

static uint8_t packet[576];
static uint16_t adc[ADC_CHANNELS];

static void process_adc(uint16_t *data)
{
	uint8_t ch;

	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

	for (ch = 0; ch < ADC_CHANNELS; ch++)
		adc[ch] = data[ch];
}

bool core_init(void)
{
	if (!adc_init(&hadc1, ADC_CHANNELS, process_adc))
		return false;

	if (!led_init(&htim4));
		return false;

	usb_printf("DMX receiver started\r\n");

	return true;
}

void core_process(void)
{
	uint16_t len;

	len = dmx_receive(packet);

	led_set(0, adc[0]);
	led_set(1, adc[1]);

	if ((adc[0] < ADC_MAX) && (adc[1] < ADC_MAX))
		usb_dumppacket(packet, len);
	else
		usb_printf("DMX.Len=%d ADC1=%d ADC2=%d\r\n",
			   len, adc[0], adc[1]);
}
