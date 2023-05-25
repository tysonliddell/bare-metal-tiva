#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "mcu.h"

static volatile uint32_t s_ticks;
void SysTick_Handler(void) { s_ticks++; }

extern int _write(int fd, char *ptr, int len);
char buffer[100];

int main(void) {
  setup_80mhz_pll_clock();
  systick_init();
  periodic_timer_init(0);

  uint8_t led_pin = 1;
  gpio_set_mode(GPIO_PIN('F', led_pin), GPIO_MODE_OUTPUT);

  uart_init(0, 115200);   // io retargeting sends printf to UART0

  uint32_t timer_expiry = 0, period_ms = 500;
  bool led_is_on = false;
  for (;;) {
    if (timer_expired(&timer_expiry, period_ms, s_ticks)) {
      led_is_on = !led_is_on;
      gpio_set_pin(GPIO_PIN('F', led_pin), led_is_on);
      printf("LED_STATE: %d, tick: %lu\r\n", led_is_on, s_ticks);

      printf("TIMER: %lu\r\n", get_timer_value(TIMER0));
    }
  }
  return 0;
}
