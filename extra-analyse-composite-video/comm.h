#pragma once

#include <netinet/in.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "crc32.h"
#include "mcu.h"

#define NAK_BYTE (0x88)
#define ACK_BYTE (0xFF)

// TX on UART until ACK byte received
void tx_until_ack(uint8_t *buffer, size_t len) {
  bool packet_acked = false;
  while (!packet_acked) {
    uart_write_buf(UART0, (char *)buffer, len);

    while (get_bit(&UART0->UARTFR, 4)) {
      // wait for UART RX FIFO to become non-empty
      (void)0;
    }

    uint8_t data = UART0->UARTDR & 0xFF;
    if (data == ACK_BYTE) {
      return;
    }
  }
}

/*
 * Send video field on UART with following simple, probably not robust,
 * protocol:
 *  - Tx a protocol packet (contains an entire scanline):
 *    +-----------------+--------------------+---------------+---------------+
 *    | scanline_number | num_scanline_bytes | scanline_data | CRC32 checksum|
 *    +-----------------+--------------------+---------------+---------------+
 *  - Wait for response from the recipient and then:
 *     - If ACK (0xFF) received, increase sequence number and send packet for
 *       next scanline
 *     - If response is anything other than `0xFF`, resend the same packet.
 */
void tx_field_results(uint8_t **scanlines, size_t num_scanlines,
                      size_t scanline_len) {
  uint8_t buffer[4 + 4 + scanline_len + 4];

  uint8_t *p = &buffer[0];
  for (uint32_t scanline_num = 0; scanline_num < num_scanlines;
       scanline_num++) {
    uint32_t netw_scanline_num = htonl(scanline_num);
    uint32_t netw_num_scanline_bytes = htonl(scanline_len);
    uint8_t *scanline = scanlines[scanline_num];

    memcpy(p, &netw_scanline_num, sizeof(uint32_t));
    p += sizeof(uint32_t);

    memcpy(p, &netw_num_scanline_bytes, sizeof(uint32_t));
    p += sizeof(uint32_t);

    memcpy(p, scanline, scanline_len * sizeof(uint8_t));
    p += scanline_len * sizeof(uint8_t);

    uint32_t netw_checksum = htonl(crc32k_lsb_first(buffer, p - buffer));
    memcpy(p, &netw_checksum, sizeof(uint32_t));

    tx_until_ack(buffer, sizeof(buffer));
  }
}
