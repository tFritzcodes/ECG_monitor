/*
 * uart.c
 *
 * Created: 5/4/2026 3:09:34 PM
 *  Author: fritz
 */ 
/* =========================================================================
 * UART  (115200 baud, U2X=1 double-speed for accuracy at 16 MHz)
 * UBRR = F_CPU / (8 * BAUD) - 1 = 16000000 / 921600 - 1 = 16.36 -> 16
 * Actual baud = 16000000 / (8 * 17) = 117647  (error ~2.1%, within spec)
 * ========================================================================= */
#include <avr/io.h>
#include "uart.h"

#define F_CPU     16000000UL

#define BAUD      115200UL
#define UBRR_VAL  ((F_CPU + 4UL * BAUD) / (8UL * BAUD) - 1)

void uart_init(void)
{
	UBRR0H  = (uint8_t)(UBRR_VAL >> 8);
	UBRR0L  = (uint8_t)(UBRR_VAL);
	UCSR0A |= (1 << U2X0);
	UCSR0B  = (1 << RXEN0) | (1 << TXEN0);
	UCSR0C  = (1 << UCSZ01) | (1 << UCSZ00);
}

void uart_putc(char c)
{
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = (uint8_t)c;
}

void uart_puts(const char *s)
{
	while (*s) uart_putc(*s++);
}

uint8_t uart_rx_ready(void)
{
	return (UCSR0A & (1 << RXC0));
}

char uart_getc(void)
{
	return (char)UDR0;
}