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
#define TIMER_PERIOD 65535

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
uint8_t PacketFlag;

uint8_t DataCorruptFlag;

void ResetPacket(void)
{

}

void rising_edge(void)
{
	uint32_t NetCounter;

	/* it is rising edge __/ . It is end of Break and start of MAB  */

	/* disable rising edge interrupt */
	__HAL_TIM_DISABLE_IT(&htim2, TIM_FLAG_CC2);
	__HAL_TIM_CLEAR_FLAG(&htim2, TIM_IT_CC2);

	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);//DEBUG

	/* Beginning of the packet */
	if (InitBreakFlag == 0)
	{
		/* Store MAB Rising Edge Counter */
		Counter2 = __HAL_TIM_GET_COUNTER(&htim2);
		/* Calculate BreakTime = MABRising - BreakFalling */
		NetCounter = (Counter2 - Counter1) & TIMER_PERIOD;
		/*
		 * Timer counter maximum value is 65535. Each overflow it
		 * increases GlobalOverflowCount, so global total time is:
		 */
		NetCounter += GlobalOverflowCount * TIMER_PERIOD;

		if (NetCounter > BreakTime) {
			BreakFlag = 1;
			InitBreakFlag = 1;
			/* enable falling edge interrupt */
			__HAL_TIM_CLEAR_FLAG(&htim2, TIM_IT_CC1);
			TIM_CCxChannelCmd(htim2.Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);
			__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC1);
			/* remember overflows at start of MAB */
			MABOverflowCount = GlobalOverflowCount;
		}
		else {
			/*
			 * Break is too short for correct DMX packet. So, edge
			 * interrupts are disabled now. Rising edge handler will
			 * be enabled again only after UART FE interrupt.
			 */
			
			GlobalOverflowCount = 0;
		}

		return;
	}

	/* It is end of Next Break */
	/* MABRising - NextBreakFalling */
	NetCounter = (Counter2 - Counter4) & TIMER_PERIOD;
	// update value of NetCounter on the basis of BreakOverflowCount
	NetCounter += BreakOverflowCount * TIMER_PERIOD;

	if (NetCounter <= BreakTime) {
		DataCorruptFlag = 1;
		return;
	}

	/* enable falling edge interrupt */
	__HAL_TIM_CLEAR_FLAG(&htim2, TIM_IT_CC1);
	__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC1);
	MABFlag = 0;
	/* for next packet */
	GlobalOverflowCount = GlobalOverflowCount - BreakOverflowCount;

	/* NextBreakFalling - BreakFalling */
	NetCounter = (Counter4 - Counter1) & 0xFFFF;
	//update the valie of NetCounter on the basis of BreakOverFlowCount
	NetCounter += BreakOverflowCount * 65535;

	BreakFlag = 1;
	InitBreakFlag = 1;
	Counter1 = Counter4;

	/* check that length of packet more than minimum */
	if (NetCounter > DMXPacketTime)
		PacketFlag = 1;
	else
		/* drop whole packet if length*/
		ResetPacket();
}

void falling_edge(void)
{
	uint32_t NetCounter;

	/* disable falling edge interrupt */
	__HAL_TIM_DISABLE_IT(&htim2, TIM_FLAG_CC1);

	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);//DEBUG.

	Counter3 = __HAL_TIM_GET_COUNTER(&htim2);//StoreMABFallingCounter
	NetCounter = (Counter3 - Counter2) & TIMER_PERIOD; //MABFalling - MABRising
	//Update value of NetCounter on basis og MABOverflowCount and GlobalOverflowCount
	NetCounter += (MABOverflowCount) * TIMER_PERIOD;

	if (BreakFlag) {
		if (NetCounter > MABTime) {
			MABFlag = 1;
			BreakFlag = 0;
		}
		else {
			GlobalOverflowCount = 0;
			BreakFlag = 0;
			InitBreakFlag = 0;
		}
	}
}

volatile uint32_t Data;
uint32_t DMXChannelCount;
uint8_t Packet[512];

void frame_error_handler(uint32_t Counter)
{
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);//DEBUG

	if ((BreakFlag == 0) && (MABFlag == 0)) {
		/* Store Break start counter */
		Counter1 = Counter;
	}
	else if (MABFlag == 1) {
		/* It is next Break, store NextBreakStart counter */
		Counter4 = Counter;
		/* Store, it will be used in ? */
		BreakOverflowCount = GlobalOverflowCount;
	}
	else
		return;

	/* enable rising edge interrupt */
	__HAL_TIM_CLEAR_FLAG(&htim2, TIM_IT_CC1);
	__HAL_TIM_CLEAR_FLAG(&htim2, TIM_IT_CC2);
	TIM_CCxChannelCmd(htim2.Instance, TIM_CHANNEL_2, TIM_CCx_ENABLE);
	__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC2);
}

void uart_handler(void)
{
	uint32_t Counter = __HAL_TIM_GET_COUNTER(&htim2);
	uint32_t StatusRead = huart1.Instance->SR;
	Data = huart1.Instance->DR;

	/* Frame Error interrupt happens after 10 bits of 0 (~40us), so it is
	 * after first 40 us of Break */
	if ((StatusRead & UART_FLAG_FE) == UART_FLAG_FE) {
		frame_error_handler(Counter);
		return;
	}

	if ((StatusRead & UART_FLAG_RXNE) == UART_FLAG_RXNE) {
		//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);//DEBUG
		if (MABFlag == 0)
			return;
		if (DataCorruptFlag == 1)
			return;

		if ((DMXChannelCount == 0) && (Data != 0x00)) {
			/* first byte must be 0x00 in dmx */
			MABFlag = 0;
			InitBreakFlag = 0;
			GlobalOverflowCount = 0;
			return;
		}

		if (DMXChannelCount < 3) {
			Packet[DMXChannelCount++] = Data;
		}
		else {
			DMXChannelCount = 0;
			//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);//DEBUG
		}
	}
}

void timer_overflow(void)
{
	GlobalOverflowCount++;
}

void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
  if __HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC1)
    falling_edge();

  if __HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC2)
    rising_edge();

  if __HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE)
    timer_overflow();

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
