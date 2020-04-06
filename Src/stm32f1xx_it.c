/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM2 global interrupt.
  */
#define TIMER_PERIOD 	0xFFFF
#define DMX_MAX_SLOTS 	512

uint32_t BreakTime = 2000;
uint32_t MABTime = 200;
uint32_t DMXPacketTime = 50000;
uint16_t Counter1;
uint16_t Counter2;
uint16_t Counter3;
uint16_t Counter4;

uint32_t GlobalOverflowCount;
uint32_t MABOverflowCount;
uint32_t BreakOverflowCount;

uint8_t InitBreakFlag;
uint8_t BreakFlag;
uint8_t MABFlag;
uint8_t DataCorruptFlag;

/* Packet byte counter, used while receiving is still in process */
uint16_t DMXChannelCount;

/* DMX Packet buffer with one byte for first 0x00 */
uint8_t Packet[DMX_MAX_SLOTS + 1];
/* Flag that indicates succesfully received packet */
uint8_t PacketFlag;
/* Length of received packet */
uint16_t PacketLength;

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
void rising_edge(void)
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
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);//DEBUG.
	}
	else {
		/* Ignore previously received packet */
		PacketLength = 0;
	}
}

/* Falling edge is end of MAB */
void falling_edge(void)
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
void frame_error_handler(uint32_t Counter)
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

void receive_data_handler(uint32_t Data)
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
	if (DMXChannelCount > DMX_MAX_SLOTS) {
		DMXChannelCount = 0;
		return;
	}

	/* Save slot data into Packet while */
	Packet[DMXChannelCount++] = Data;
}

void uart_handler(void)
{
	uint32_t Counter = __HAL_TIM_GET_COUNTER(&htim2);
	uint32_t StatusRead = huart1.Instance->SR;
	uint32_t Data = huart1.Instance->DR;

	/* Frame Error interrupt happens after 10 bits of 0 (~40us), so it is
	 * after first 40 us of Break */
	if ((StatusRead & UART_FLAG_FE) == UART_FLAG_FE) {
		frame_error_handler(Counter);
		__HAL_UART_CLEAR_FEFLAG(&huart1);
		return;
	}

	/* Receive Data interrupt */
	if ((StatusRead & UART_FLAG_RXNE) == UART_FLAG_RXNE) {
		receive_data_handler(Data);
		__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RXNE);
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

void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
  uart_handler();
  /* USER CODE END USART1_IRQn 0 */
  //HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
