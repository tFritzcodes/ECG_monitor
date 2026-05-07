/*
 * uart.h
 *
 * Created: 5/4/2026 3:09:50 PM
 *  Author: fritz
 */ 

#ifndef UART_H
#define UART_H

#include <stdint.h>

void uart_init(void);
void uart_putc(char c);
void uart_puts(const char *s);
uint8_t uart_rx_ready(void);
char uart_getc(void);

#endif