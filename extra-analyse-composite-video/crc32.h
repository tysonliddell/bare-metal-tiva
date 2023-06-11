#pragma once
/*
 * Algorithm tested with:
 * void main(void) {
 *     char data[] = "123456789";
 *     uint32_t crc = crc32c_lsb_first(data, 9);
 *     printf("%x\n", crc);
 * }

 * Output was 0xe3069283, which is expected for CRC-32C according to:
 * https://reveng.sourceforge.io/crc-catalogue/17plus.htm#crc.cat.crc-32c
 *
 * CRC-32/ISCSI (AKA CRC-32C)
 *   width=32 poly=0x1edc6f41 init=0xffffffff refin=true refout=true
 *   xorout=0xffffffff check=0xe3069283 residue=0xb798b438 name="CRC-32/ISCSI"
 * See https://reveng.sourceforge.io/readme.htm for the meaning of these
 * parameters or
 *     https://reveng.sourceforge.io/crc-catalogue/legend.htm#crc.legend.params
 *
 * The algorithm described in the '8. A Straightforward CRC Implementation'
 * section of
 * https://ceng2.ktu.edu.tr/~cevhers/ders_materyal/bil311_bilgisayar_mimarisi/supplementary_docs/crc_algorithms.pdf,
 * generates the check value above as follows:
 * - Uses a right-to-left shift register
 * - Feeds the message polynomial LSBit first
 * - Uses a non-reversed crc polynomial
 * - Results in the actual value of the remainder after dividing the message
 *   polynomial by the crc polynomial
 * - Then reflects the final remainder polynomial
 *
 * The algorithm implemented below conceptually does the same thing, but
 * reflects the whole process from the beginning, giving the same result in the
 * end:
 * - Uses a left-to-right shift register
 * - Feeds the message polynomial LSBit first
 * - Uses a reversed crc polynomial
 * - Results in the reflected remainder polynomial
 */

#include <stddef.h>
#include <stdint.h>

#define POLYNOMIAL_CRC32C (0x1EDC6F41)
#define POLYNOMIAL_CRC32C_REVERSED (0x82F63B78)
#define POLYNOMIAL_CRC32K_6_4 (0x32C00699)
#define POLYNOMIAL_CRC32K_6_4_REVERSED (0x9960034C)
#define POLYNOMIAL_CRC16_CCITT (0x1021)

/*
 * Compute a 32-bit CRC checksum for data to be sent one byte at a time, least
 * significant bit first (UART).
 */
uint32_t crc32k_lsb_first(uint8_t *buffer, size_t buffer_size) {
  uint32_t rem = 0xFFFFFFFF;

  for (size_t i = 0; i < buffer_size; i++) {
    rem ^= buffer[i];
    for (int j = 0; j < 8; j++) {
      if (rem & 0x00000001) {
        rem = (rem >> 1) ^ POLYNOMIAL_CRC32K_6_4_REVERSED;
      } else {
        rem >>= 1;
      }
    }
  }

  return rem ^ 0xFFFFFFFF;
}

/*
 * Compute a 16-bit CRC checksum for xmodem:
 *   width=16 poly=0x1021 init=0x0000 refin=false refout=false xorout=0x0000
 *   check=0x31c3 residue=0x0000 name="CRC-16/XMODEM"
 */
uint16_t crc16_xmodem(uint8_t *buffer, size_t buffer_size) {
  uint16_t rem = 0x0;

  for (size_t i = 0; i < buffer_size; i++) {
    rem ^= (buffer[i] << (16 - 8));
    for (int j = 0; j < 8; j++) {
      if (rem & 0x8000) {
        rem = (rem << 1) ^ POLYNOMIAL_CRC16_CCITT;
      } else {
        rem <<= 1;
      }
    }
  }

  return rem;
}
