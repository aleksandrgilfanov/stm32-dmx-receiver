#include <stdbool.h>
#include <string.h>

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f1xx_hal_uart.h"

#include "dmx_receiver.h"

#define TIMER_PERIOD 	0xFFFF
#define DMX_MAX_SLOTS 	512

extern TIM_HandleTypeDef htim2;

static uint32_t BreakTime = 2000;
static uint32_t MABTime = 200;
static uint32_t DMXPacketTime = 50000;

static uint16_t Counter1;
static uint16_t Counter2;
static uint16_t Counter3;
static uint16_t Counter4;

static uint32_t GlobalOverflowCount;
static uint32_t MABOverflowCount;
static uint32_t BreakOverflowCount;

static uint8_t InitBreakFlag;
static uint8_t BreakFlag;
static uint8_t MABFlag;
static uint8_t DataCorruptFlag;

/* Packet byte counter, used while receiving is still in process */
static uint16_t DMXChannelCount;

/* DMX Packet buffer with one byte for first 0x00 */
static uint8_t Packet[DMX_MAX_SLOTS + 1];
/* Flag that indicates succesfully received packet */
static uint8_t PacketFlag;
/* Length of received packet */
static uint16_t PacketLength;

/* Blocking receive entire DMX packet */
uint16_t dmx_receive(uint8_t* dest)
{
	uint16_t len;

	while (PacketFlag == 0) {};

	len = PacketLength;
	memcpy(dest, Packet, len);
	PacketFlag = 0;

	return len;
}

static void first_break_rising(uint32_t OverflowCount)
{
	uint32_t NetCounter;

	/* Calculate BreakTime = MABRising - BreakFalling */
	NetCounter = (Counter2 - Counter1) & TIMER_PERIOD;
	/*
	 * Timer counter maximum value is 65535. Each overflow it
	 * increases GlobalOverflowCount, so global total time is:
	 */
	NetCounter += OverflowCount * TIMER_PERIOD;

	if (NetCounter > BreakTime) {
		/* Set flags that Break is detected */
		BreakFlag = 1;
		InitBreakFlag = 1;

		/* Enable falling edge interrupt */
		__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC1);

		/* Remember overflows at start of MAB */
		MABOverflowCount = OverflowCount;
	}
	else {
		/*
		 * Break is too short for correct DMX packet. So, edge
		 * interrupts are disabled now. Rising edge handler will
		 * be enabled again only after UART FE interrupt.
		 */
		GlobalOverflowCount = 0;
	}
}

static bool next_break_rising(uint32_t OverflowCount)
{
	uint32_t NetCounter;

	/* MABRising - NextBreakFalling */
	NetCounter = (Counter2 - Counter4) & TIMER_PERIOD;
	/* with regard to GlobalOverflowCount and BreakOverflowCount  */
	NetCounter += (OverflowCount - BreakOverflowCount) * TIMER_PERIOD;

	/* If Break is too short */
	if (NetCounter <= BreakTime) {
		DataCorruptFlag = 1;
		return false;
	}

	/* Enable falling edge interrupt */
	__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC1);
	/* Reset old MAB flag, falling edge interrupt will set it if found */
	MABFlag = 0;
	/* For next packet */
	GlobalOverflowCount = GlobalOverflowCount - BreakOverflowCount;

	/* Calculate time between Breaks = NextBreakFalling - BreakFalling */
	NetCounter = (Counter4 - Counter1) & TIMER_PERIOD;
	/* Total time beetween Breaks */
	NetCounter += BreakOverflowCount * TIMER_PERIOD;

	BreakFlag = 1;
	InitBreakFlag = 1;
	Counter1 = Counter4;

	/* If Break to Break time is correct set flag */
	if (NetCounter < DMXPacketTime)
		return false;

	return true;
}

/* It is rising edge __/ . It is end of Break and start of MAB  */
static void rising_edge(void)
{
	uint32_t OverflowCount = GlobalOverflowCount;

	/* Store MAB Rising Edge Counter */
	Counter2 = __HAL_TIM_GET_COUNTER(&htim2);

	/* Disable rising edge interrupt */
	__HAL_TIM_DISABLE_IT(&htim2, TIM_FLAG_CC2);

	/* No Breaks received yet, it is first */
	if (InitBreakFlag == 0)
	{
		/* Check First Break time etc. */
		first_break_rising(OverflowCount);
		return;
	}

	/*
	 * Check Next Break time, then check Break to Break. If it is correct,
	 * then previous received packet can be used
         */
	if (next_break_rising(OverflowCount))
	{
		/* Set Flag that Packet is ready */
		PacketFlag = 1;
	}
	else {
		/* Ignore previously received packet */
		PacketLength = 0;
	}
}

/* Falling edge is end of MAB */
static void falling_edge(void)
{
	uint32_t NetCounter;
	uint32_t OverflowCount = GlobalOverflowCount;

	/* Disable falling edge interrupt */
	__HAL_TIM_DISABLE_IT(&htim2, TIM_FLAG_CC1);

	/* It is MAB Falling edge, so store MAB End counter value */
	Counter3 = __HAL_TIM_GET_COUNTER(&htim2);

	/* Calculate MAB Time = MABFalling - MABRising */
	NetCounter = (Counter3 - Counter2) & TIMER_PERIOD;
	/* Total time */
	NetCounter += (OverflowCount - MABOverflowCount) * TIMER_PERIOD;

	/* Return if correct Break was not detected previously */
	if (BreakFlag == 0)
		return;

	if (NetCounter > MABTime) {
		/* Got correct MAB, ready to receive slots */
		MABFlag = 1;
		BreakFlag = 0;
	}
	else {
		/* Too short MAB, so clear and wait for new Break (UART FE) */
		GlobalOverflowCount = 0;
		BreakFlag = 0;
		InitBreakFlag = 0;
	}
}

/* Frame Error is detected by UART after 10bits of low level. So it is Break */
static void frame_error_handler(uint32_t Counter)
{
	/* actually FE is 40 usec after Break start... */

	if ((BreakFlag == 0) && (MABFlag == 0)) {
		/* Store first Break start counter */
		Counter1 = Counter;
	}
	else if (MABFlag == 1) {
		/* It is next Break, store NextBreakStart counter */
		Counter4 = Counter;
		/* Store Overflows, it will be used to calculate time between Breaks */
		BreakOverflowCount = GlobalOverflowCount;
	}
	else
		return;

	/*
	 * Save already received packet length, but it must be used only when
	 * PacketFlag will be set (after checking Break time)
	 */
	PacketLength = DMXChannelCount;
	/* Prepare counter for receiving slots of next packet */
	DMXChannelCount = 0;

	/* Enable rising edge interrupt, to detect Break End */
	__HAL_TIM_CLEAR_FLAG(&htim2, TIM_IT_CC1);
	__HAL_TIM_CLEAR_FLAG(&htim2, TIM_IT_CC2);
	__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC2);
}

static void receive_data_handler(uint32_t Data)
{
	/* Do not receice data without correct MAB */
	if (MABFlag == 0)
		return;
	/*
	 * MABFlag is still set at second MAB. So, MAB Falling interrupt
	 * sets DataCorruptFlag if second MAB is too short
	 */
	if (DataCorruptFlag == 1)
		return;

	/* Drop packet if first byte is not 0x00 */
	if ((DMXChannelCount == 0) && (Data != 0x00)) {
		MABFlag = 0;
		InitBreakFlag = 0;
		GlobalOverflowCount = 0;
		return;
	}

	/* Drop all packet if it is too long */
	if (DMXChannelCount > (DMX_MAX_SLOTS + 1)) {
		DMXChannelCount = 0;
		return;
	}

	/* Save slot data into Packet while */
	Packet[DMXChannelCount++] = Data;
}

void dmx_uart_handler(UART_HandleTypeDef *huart)
{
	uint32_t Counter = __HAL_TIM_GET_COUNTER(&htim2);
	uint32_t StatusRead = huart->Instance->SR;
	uint32_t Data = huart->Instance->DR;

	/* Frame Error interrupt happens after 10 bits of 0 (~40us), so it is
	 * after first 40 us of Break */
	if ((StatusRead & UART_FLAG_FE) == UART_FLAG_FE) {
		frame_error_handler(Counter);
		__HAL_UART_CLEAR_FEFLAG(huart);
		return;
	}

	/* Receive Data interrupt */
	if ((StatusRead & UART_FLAG_RXNE) == UART_FLAG_RXNE) {
		receive_data_handler(Data);
		__HAL_UART_CLEAR_FLAG(huart, UART_FLAG_RXNE);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	GlobalOverflowCount++;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	falling_edge();
  else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	rising_edge();
}
