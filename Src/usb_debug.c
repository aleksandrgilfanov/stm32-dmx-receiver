#include <stdarg.h>
#include "stdio.h"
#include <string.h>

#include "usb_debug.h"
#include "usbd_cdc_if.h"

static char usb_buf[512];

void usb_printf(char *fmt, ...)
{
	uint16_t len;
	va_list ap;

	va_start(ap, fmt);
	vsprintf(usb_buf, fmt, ap);
	va_end(ap);

	len = strlen(usb_buf);

	CDC_Transmit_FS((uint8_t*)usb_buf, len);
}

void usb_dumppacket(uint8_t *src_packet, uint16_t len)
{
	char *ptr = usb_buf;
	uint16_t to_send;

	ptr += sprintf(ptr, "[%lu] Dumping packet (type=%02X len=%d):",
		       HAL_GetTick(), src_packet[0], len);

	for (int i = 1; i < len; i++) {
		if ((i % 16) == 1)
			ptr += sprintf(ptr, "\r\n");

		ptr += sprintf(ptr, "%02X ", src_packet[i]);

		to_send = ptr - usb_buf;
		if (to_send > 500)
		{
			CDC_Transmit_FS((uint8_t*)usb_buf, to_send);
			ptr = usb_buf;
		}
	}

	ptr += sprintf(ptr, "\r\n");

	to_send = ptr - usb_buf;
	CDC_Transmit_FS((uint8_t*)usb_buf, to_send);
}
