#include <string.h>

#include "dmx_receiver.h"

extern uint8_t Packet[];
extern uint8_t PacketFlag;
extern uint16_t PacketLength;

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

