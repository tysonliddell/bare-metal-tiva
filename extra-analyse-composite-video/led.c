#include "mcu.h"
#include "led.h"

void enable_leds(void) {
  gpio_set_mode(RED_PIN, GPIO_MODE_DIGITAL_OUTPUT);
  gpio_set_mode(BLUE_PIN, GPIO_MODE_DIGITAL_OUTPUT);
  gpio_set_mode(GREEN_PIN, GPIO_MODE_DIGITAL_OUTPUT);
}

void set_led_state(LedState state) {
  gpio_set_pin(RED_PIN, false);
  gpio_set_pin(BLUE_PIN, false);
  gpio_set_pin(GREEN_PIN, false);

  switch (state) {
  case LED_STATE_RED:
    gpio_set_pin(RED_PIN, true);
    break;
  case LED_STATE_GREEN:
    gpio_set_pin(GREEN_PIN, true);
    break;
  case LED_STATE_BLUE:
    gpio_set_pin(BLUE_PIN, true);
    break;
  case LED_STATE_YELLOW:
    gpio_set_pin(RED_PIN, true);
    gpio_set_pin(GREEN_PIN, true);
    break;
  case LED_STATE_WHITE:
    gpio_set_pin(RED_PIN, true);
    gpio_set_pin(GREEN_PIN, true);
    gpio_set_pin(BLUE_PIN, true);
  default:
    break;
  }
}

