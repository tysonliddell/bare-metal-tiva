#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "mcu.h"

static volatile uint32_t s_ticks;
void SysTick_Handler(void) { s_ticks++; }

void comparator_count_low_handler(void) {
  static uint32_t start_ticks;

  const uint32_t current_ticks = get_timer_value(TIMER0);
  const bool is_ac_inverted = get_bit(&AC->ACCTL0, 1);
  if (!is_ac_inverted) {
    start_ticks = current_ticks;
    printf("Comparator went low at tick %lu\r\n", start_ticks);
    set_bit(&AC->ACCTL0, 1); // invert comparator output
  } else {
    uint32_t ticks_elapsed = current_ticks - start_ticks;
    uint32_t millis_elapsed = ticks_to_microseconds(ticks_elapsed) / 1000;
    printf("Comparator was low for %lu ms\r\n", millis_elapsed);
    clear_bit(&AC->ACCTL0, 1); // uninvert comparator output
  }

  set_bit(&AC->ACMIS, 0); // clear the interrupt
}

void setup_composite_video_experiment(void) {
  // Configure the analog comparator
  gpio_set_mode(GPIO_PIN('C', 6), GPIO_MODE_ANALOG_INPUT);
  gpio_set_mode(GPIO_PIN('C', 7), GPIO_MODE_ANALOG_INPUT);
  gpio_enable_analog_function(GPIO_PIN('C', 6));
  gpio_enable_analog_function(GPIO_PIN('C', 7));

  RCGC->RCGCACMP = 0x1; // give AC a clock
  spin(3);
  set_bit(
      &AC->ACINTEN,
      0); // enable interrupt on AC0  (vector number 41, interrupt number 25)
  set_bit(&NVIC->EN0, 25);

  // set reference voltage to ~100 mV, assume any reading less than this is a
  // 0 V sync pulse.
  set_bit(&AC->ACREFCTL, 9); // enable reference voltage
  set_bit(&AC->ACREFCTL, 8); // use low range

  AC->ACREFCTL &= ~0xFU; // set VREF to encoding 0x1 (149 mV)
  AC->ACREFCTL |= 0x1;

  AC->ACCTL0 &= ~(0x3U << 9); // use internal reference voltage
  AC->ACCTL0 |= (0x2 << 9);

  // generate an interrupt when comparator output is high (sampled voltage is
  // low)
  set_bit(&AC->ACCTL0, 4);
}

int main(void) {
  setup_80mhz_pll_clock();
  systick_init();
  periodic_timer_init(0);
  enable_fpu();
  uart_init(0, 115200); // io retargeting sends printf to UART0

  setup_composite_video_experiment();

  // setup debug LED
  uint8_t led_pin = 1;
  gpio_set_mode(GPIO_PIN('F', led_pin), GPIO_MODE_DIGITAL_OUTPUT);

  uint32_t timer_expiry = 0, period_ms = 500;
  bool led_is_on = false;
  for (;;) {
    if (timer_expired(&timer_expiry, period_ms, s_ticks)) {
      led_is_on = !led_is_on;
      gpio_set_pin(GPIO_PIN('F', led_pin), led_is_on);
      // printf("LED_STATE: %d, tick: %lu\r\n", led_is_on, s_ticks);
      // printf("TIMER: %lu\r\n", get_timer_value(TIMER0));
      // printf("COMPARATOR: %lu\r\n", AC->ACSTAT0);
    }
  }
  return 0;
}
