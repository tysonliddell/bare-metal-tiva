#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "mcu.h"

static volatile uint32_t s_ticks;
void SysTick_Handler(void) { s_ticks++; }

extern int _write(int fd, char *ptr, int len);
char buffer[100];

int main(void) {
  systick_init();
  gpio_set_mode(GPIO_PIN('F', 1), GPIO_MODE_OUTPUT);
  gpio_set_mode(GPIO_PIN('F', 2), GPIO_MODE_OUTPUT);
  gpio_set_mode(GPIO_PIN('F', 3), GPIO_MODE_OUTPUT);

  uart_init(0, 115200);   // io retargeting sends printf to UART0

  uint32_t timer_expiry = 0, period_ms = 500;
  uint32_t state = 0;
  for (;;) {
    if (timer_expired(&timer_expiry, period_ms, s_ticks)) {
      bool led_state = !(state & 0x1);
      uint32_t led_pin = state / 2 + 1;
      gpio_set_pin(GPIO_PIN('F', led_pin), led_state);
      state = state == 5 ? 0 : state + 1;

      printf("LED: %lu, tick: %lu\r\n", state, s_ticks);
    }
  }
  return 0;
}
