#pragma once

#include <stddef.h>
#include <stdint.h>

/*
 * An minimal implementation of the XMODEM protocol to send data from the MCU to
 * a receiver.
 *
 * Listen for a NAK from the receiver and then send a stream of data on UART.
 * Uses a 32-bit CRC.
 */
void xmodem_wait_and_send(const uint8_t *data, const size_t len);
