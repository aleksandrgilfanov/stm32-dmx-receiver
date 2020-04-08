#ifndef DMX_RECEIVER_H
#define DMX_RECEIVER_H

#include <stdint.h>

#define DMX_BREAK	92
#define DMX_MAB		12
#define DMX_SLOT	44
#define DMX_MBS		1

uint16_t dmx_receive(uint8_t* dest);
void dmx_uart_handler(UART_HandleTypeDef *huart);

#endif		/* DMX_RECEIVER_H */
