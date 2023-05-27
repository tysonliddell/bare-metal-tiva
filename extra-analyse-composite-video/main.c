#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "mcu.h"

#define NUM_LOW_MEASUREMENTS (1000)

static volatile uint32_t s_ticks;
void SysTick_Handler(void) { s_ticks++; }

static volatile uint32_t pulse_durations[NUM_LOW_MEASUREMENTS];
static volatile uint32_t pulse_occurred_at_tick[NUM_LOW_MEASUREMENTS];
static volatile uint32_t pulse_count = 0;

void print_results(void) {
    printf("Sync pulse lengths:\r\n");
    int i = 1;  // start at 1 so we can always compare to a previous pulse
    while (pulse_durations[i] != 2) {
      // skip to start of next frame (short-sync pulse is 2 µs)
      i++;
    }
    // uint32_t first_pulse_tick = pulse_occurred_at_tick[i];
    for (; i < NUM_LOW_MEASUREMENTS; i++) {
      // uint32_t occurred_at = ticks_to_microseconds(pulse_occurred_at_tick[i] - first_pulse_tick);
      // printf("len: %lu us, when: %lu us\r\n", pulse_durations[i], occurred_at);
      uint32_t deltat = ticks_to_microseconds(pulse_occurred_at_tick[i] - pulse_occurred_at_tick[i-1]);
      printf("len: %lu us, time since last pulse: %lu us\r\n", pulse_durations[i], deltat);
    }
    printf("\r\n");
    printf("\r\n");
}

void comparator_count_low_handler(void) {
  static uint32_t start_ticks;

  const uint32_t current_ticks = get_timer_value(TIMER0);
  const bool is_ac_inverted = get_bit(&AC->ACCTL1, 1);
  if (!is_ac_inverted) {
    start_ticks = current_ticks;
    set_bit(&AC->ACCTL1, 1); // invert comparator output
  } else {
    uint32_t ticks_elapsed = current_ticks - start_ticks;
    uint32_t micros_elapsed = ticks_to_microseconds(ticks_elapsed);
    pulse_occurred_at_tick[pulse_count] = current_ticks;
    pulse_durations[pulse_count] = micros_elapsed;
    pulse_count++;
    clear_bit(&AC->ACCTL1, 1); // uninvert comparator output
  }
  if (pulse_count == NUM_LOW_MEASUREMENTS) {
    print_results();
    pulse_count = 0;

    // disable the comparator interrupt
    set_bit(&NVIC->DIS0, 26);
  }

  set_bit(&AC->ACMIS, 1); // clear the interrupt
}

void setup_composite_video_experiment(void) {
  // Configure the analog comparator
  gpio_set_mode(GPIO_PIN('C', 4), GPIO_MODE_ANALOG_INPUT);
  gpio_set_mode(GPIO_PIN('C', 5), GPIO_MODE_ANALOG_INPUT);
  gpio_enable_analog_function(GPIO_PIN('C', 4));
  gpio_enable_analog_function(GPIO_PIN('C', 5));

  RCGC->RCGCACMP = 0x1; // give AC a clock
  spin(3);
  set_bit(
      &AC->ACINTEN,
      1); // enable interrupt on AC1  (vector number 42, interrupt number 26)
  set_bit(&NVIC->EN0, 26);

  // set reference voltage to ~100 mV, assume any reading less than this is a
  // 0 V sync pulse.
  set_bit(&AC->ACREFCTL, 9); // enable reference voltage
  set_bit(&AC->ACREFCTL, 8); // use low range

  AC->ACREFCTL &= ~0xFU; // set VREF to encoding 0x1 (149 mV)
  AC->ACREFCTL |= 0x1;

  AC->ACCTL1 &= ~(0x3U << 9); // use internal reference voltage
  AC->ACCTL1 |= (0x2 << 9);

  // generate an interrupt when comparator output is high (sampled voltage is
  // low)
  set_bit(&AC->ACCTL1, 4);
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
