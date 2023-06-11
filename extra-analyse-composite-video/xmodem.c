/*
 * An minimal implementation of the XMODEM protocol to send data from the MCU to
 * a receiver. Probably overkill, but uses a 32-bit checksum.
 */
#include <machine/endian.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "crc32.h"
#include "led.h"
#include "mcu.h"
#include "xmodem.h"

#define SOH ((uint8_t)0x01)
#define ACK ((uint8_t)0x06)
#define NAK ((uint8_t)0x15)
#define CRC_START ((uint8_t)'C')
#define EOT ((uint8_t)0x04)
#define PADDING ((uint8_t)0x1A)

#define SOH_SIZE (1)
#define BLOCK_COUNTER_SIZE (2)
#define BLOCK_SIZE (128)
#define CHECKSUM_SIZE (2)
#define HEADER_SIZE (SOH_SIZE + BLOCK_COUNTER_SIZE)
#define PACKET_SIZE (HEADER_SIZE + BLOCK_SIZE + CHECKSUM_SIZE)

#define NUM_TX_RETRIES (10)

#define MIN(A, B) ((A) < (B) ? (A) : (B))

static void send(const uint8_t *data, const size_t len);
static bool send_block(const uint8_t *data, const size_t len,
                       const uint8_t packet_num);
static bool send_bytes(const uint8_t *bytes, const size_t len,
                       const uint32_t num_attempts);
static bool wait_for_byte(const uint8_t expected_byte);

/*
 * Listen for a 'C' byte from the receiver and then send a stream of data on
 * UART using XMODEM-crc protocol. Does not fallback to standard non-crc XMODEM
 * protocol if receiver does not support it. I.e. if the receiver initiates
 * communication with a NAK rather than a 'C'.
 */
void xmodem_wait_and_send(const uint8_t *data, const size_t len) {
  set_led_state(LED_STATE_YELLOW);
  while (!wait_for_byte(CRC_START)) {
    // wait for receiver to send a NAK byte.
    (void)0;
  }

  send(data, len);
}

/*
 * Break data into a sequence of blocks and send each to the receiver.
 */
static void send(const uint8_t *data, const size_t len) {
  set_led_state(LED_STATE_BLUE);

  static const uint8_t eot = EOT;
  const size_t total_blocks = (len / BLOCK_SIZE) + 1;

  for (uint8_t block = 0; block < total_blocks; block++) {
    if (!send_block(data, len, block)) {
      // Failed to send packet. Bail!
      set_led_state(LED_STATE_RED);
      return;
    }
  }

  // send EOT and wait for ACK
  if (send_bytes(&eot, 1, NUM_TX_RETRIES)) {
    set_led_state(LED_STATE_GREEN);
  } else {
    // EOT byte not ACKed
    set_led_state(LED_STATE_RED);
  }
}

/*
 * Send a single packet containing a block. Packet numbers start at 1. Returns
 * true if the packet is ACKed by the receiver and false if NAKed.
 */
static bool send_block(const uint8_t *data, const size_t len,
                       const uint8_t packet_num) {
  static uint8_t packet[PACKET_SIZE];

  // headers
  packet[0] = SOH;
  packet[1] = packet_num + 1; // packets are 1-indexed by protocol
  packet[2] = 255 - packet[1];

  const size_t start_byte_idx = packet_num * BLOCK_SIZE;
  const size_t bytes_remaining = len - start_byte_idx;
  const size_t data_payload_size = MIN(bytes_remaining, BLOCK_SIZE);

  // block of data
  memset(&packet[HEADER_SIZE], PADDING, BLOCK_SIZE);
  memcpy(&packet[HEADER_SIZE], &data[start_byte_idx], data_payload_size);

  uint16_t checksum = __htons(crc16_xmodem(&packet[HEADER_SIZE], BLOCK_SIZE));
  memcpy(&packet[PACKET_SIZE - CHECKSUM_SIZE], &checksum, CHECKSUM_SIZE);

  return send_bytes(packet, PACKET_SIZE, NUM_TX_RETRIES);
}

/*
 * Send bytes and wait for ACK. Return true if ACKed within `num_attempts`
 * attempts, false otherwise.
 */
static bool send_bytes(const uint8_t *bytes, const size_t len,
                       const uint32_t num_attempts) {
  bool bytes_acked = false;
  for (uint32_t i = 0; i < num_attempts; i++) {
    uart_write_buf(UART0, (char *)bytes, len);

    // flush the UART receive buffer
    while (!get_bit(&UART0->UARTFR, 4)) {
      UART0->UARTDR;
    }

    bytes_acked = wait_for_byte(ACK);
    if (bytes_acked) {
      return true;
    }
  }

  return false;
}

/*
 * Poll UART forever until a byte is received.
 * Return true if byte is expected, false otherwise.
 */
static bool wait_for_byte(const uint8_t expected_byte) {
  while (get_bit(&UART0->UARTFR, 4)) {
    // wait for UART RX FIFO to become non-empty
    (void)0;
  }

  const uint8_t data = UART0->UARTDR & 0xFF;
  return data == expected_byte;
}
