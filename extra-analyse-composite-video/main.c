#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "led.h"
#include "mcu.h"
#include "xmodem.h"

#define MAX_VIDEO_SAMPLES (30000) // allows 30 per scanline and 1000 scanlines
#define MAX_NUM_SCANLINES (1000)
#define NUM_SAMPLES_PER_SCANLINE (30)
#define H_SYNC_PULSE_DURATION_MICROSECONDS (4)
#define LONG_SYNC_PULSE_DURATION_MICROSECONDS (27)
#define ACTIVE_VIDEO_DURATION_MICROSECONDS                                     \
  (52 - 5) // TODO: as close to 52 as possible

#define ADC_MIN_uVOLTS (0)
#define ADC_MAX_uVOLTS (3300000)
#define ADC_NUM_DISCRETE_SAMPLES (0x1000)
#define ADC_SAMPLE_DELTA_uV                                                    \
  ((ADC_MAX_uVOLTS - ADC_MIN_uVOLTS) / (ADC_NUM_DISCRETE_SAMPLES))
#define ADC_SAMPLE_300mV ((300000 - ADC_MIN_uVOLTS) / ADC_SAMPLE_DELTA_uV)
#define ADC_SAMPLE_700mV ((700000 - ADC_MIN_uVOLTS) / ADC_SAMPLE_DELTA_uV)
#define ADC_SAMPLE_MAX (0xFFF)

static volatile uint32_t s_ticks;
void SysTick_Handler(void) { s_ticks++; }

static uint8_t scanline_samples[MAX_NUM_SCANLINES][NUM_SAMPLES_PER_SCANLINE];
static uint32_t scanline_count = 0;

void send_field_results(void) {
  xmodem_wait_and_send((uint8_t *)scanline_samples,
                       scanline_count * NUM_SAMPLES_PER_SCANLINE);
}

static inline bool is_long_sync_pulse_duration(uint32_t duration_microseconds) {
  return duration_microseconds >= LONG_SYNC_PULSE_DURATION_MICROSECONDS - 2 &&
         duration_microseconds <= LONG_SYNC_PULSE_DURATION_MICROSECONDS + 2;
}

static inline bool is_hsync_duration(uint32_t duration_microseconds) {
  return duration_microseconds >= H_SYNC_PULSE_DURATION_MICROSECONDS &&
         duration_microseconds <= H_SYNC_PULSE_DURATION_MICROSECONDS + 1;
}

static inline uint8_t normalise_and_truncate_12bit_sample(uint32_t sample) {
  int32_t signed_sample =
      (int32_t)sample; // this is safe since sample is in range [0x0, 0xFFF]

  // linearly map 300mV samples to 0x0 and 1000mV samples to 0xFFF.
  int32_t normalised_sample =
      (signed_sample - ADC_SAMPLE_300mV) * (ADC_SAMPLE_MAX / ADC_SAMPLE_700mV);

  // clamp sample to range [0x0, 0xFFF] ([300mV, 1000mV] in volts)
  if (normalised_sample < 0) {
    normalised_sample = 0;
  } else if (normalised_sample > ADC_SAMPLE_MAX) {
    normalised_sample = ADC_SAMPLE_MAX;
  }

  // truncate to 8-bits
  return (uint8_t)(normalised_sample >> 4);
}

void capture_scanline(uint32_t duration_microseconds) {
  const uint32_t start_ticks = get_timer_value(TIMER0);
  const uint32_t end_ticks =
      start_ticks + microseconds_to_ticks(duration_microseconds);
  uint32_t sample_count = 0;

  // sample ADC with naive approach that doesn't use DMA.
  set_bit(&ADC0->ADCACTSS, 3); // start SS3 ready for (continuous) sampling

  while (get_timer_value(TIMER0) < end_ticks &&
         sample_count < NUM_SAMPLES_PER_SCANLINE) {
    while (get_bit(&ADC0->ADCSSFSTAT3, 8)) {
      // wait for ADC buffer to be non-empty
      (void)0;
    }
    uint32_t sample = ADC0->ADCSSFIFO3; // save recorded sample
    scanline_samples[scanline_count][sample_count++] =
        normalise_and_truncate_12bit_sample(sample);
  }

  clear_bit(&ADC0->ADCACTSS, 3); // end SS3 sampling and clear stack
  if (!get_bit(&ADC0->ADCSSFSTAT3, 8) &&
      sample_count < NUM_SAMPLES_PER_SCANLINE) {
    uint32_t sample = ADC0->ADCSSFIFO3;
    scanline_samples[scanline_count][sample_count++] =
        normalise_and_truncate_12bit_sample(sample);
  }

  scanline_count++;
}

void comparator_count_low_handler(void) {
  static uint32_t start_ticks;
  static bool capture_scanlines = false;

  const uint32_t current_ticks = get_timer_value(TIMER0);
  const bool is_ac_inverted = get_bit(&AC->ACCTL1, 1);
  if (!is_ac_inverted) {
    start_ticks = current_ticks;
    set_bit(&AC->ACCTL1, 1); // invert comparator output
  } else {
    const uint32_t ticks_elapsed = current_ticks - start_ticks;
    const uint32_t micros_elapsed = ticks_to_microseconds(ticks_elapsed);
    if (is_long_sync_pulse_duration(micros_elapsed)) {
      if (scanline_count > 0) {
        capture_scanlines = false; // already captured one field
        send_field_results();
        scanline_count = 0;

        // disable the comparator interrupt
        set_bit(&NVIC->DIS0, 26);
      } else {
        capture_scanlines = true;
      }
    } else if (is_hsync_duration(micros_elapsed)) {
      if (capture_scanlines) {
        capture_scanline(ACTIVE_VIDEO_DURATION_MICROSECONDS);
      }
    }
    clear_bit(&AC->ACCTL1, 1); // uninvert comparator output
  }

  set_bit(&AC->ACMIS, 1); // clear the interrupt
}

void setup_composite_video_experiment(void) {
  // CONFIGURE ANALOG COMPARATOR (used for sync timing)
  gpio_set_mode(GPIO_PIN('C', 4), GPIO_MODE_ANALOG_INPUT);
  gpio_enable_analog_function(GPIO_PIN('C', 4));

  set_bit(&RCGC->RCGCACMP, 0); // give AC module a clock
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

  // CONFIGURE ADC (used to measure the active video signal)
  gpio_set_mode(GPIO_PIN('E', 3), GPIO_MODE_ANALOG_INPUT);
  gpio_enable_analog_function(GPIO_PIN('E', 3));

  set_bit(&RCGC->RCGCADC, 0); // give ADC0 module a clock
  spin(3);

  clear_bit(&ADC0->ADCACTSS, 3); // disable SS3 before configuring
  ADC0->ADCEMUX |= (0xF << 12);  // set SS3 to continuously sample
  ADC0->ADCSSMUX3 = 0x0;         // set SS3 to take samples from AIN0

  // configure the SS3 sample sequence
  ADC0->ADCSSCTL3 = 0x0;
  set_bit(&ADC0->ADCSSCTL0, 1); // sequence ends after one sample, no
                                // interrupts, no differential sampling
}

int main(void) {
  setup_80mhz_pll_clock();
  systick_init();
  periodic_timer_init(0);
  enable_fpu();
  uart_init(0, 115200); // io retargeting sends printf to UART0
  enable_leds();

  setup_composite_video_experiment();

  uint32_t timer_expiry = 0, period_ms = 500;
  bool led_is_on = false;
  for (;;) {
    if (timer_expired(&timer_expiry, period_ms, s_ticks)) {
      led_is_on = !led_is_on;
      led_is_on ? set_led_state(LED_STATE_RED) : set_led_state(LED_STATE_OFF);
    }
  }
  return 0;
}
