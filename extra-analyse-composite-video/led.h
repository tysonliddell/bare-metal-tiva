#pragma once

#include "mcu.h"

#define RED_PIN (GPIO_PIN('F', 1))
#define BLUE_PIN (GPIO_PIN('F', 2))
#define GREEN_PIN (GPIO_PIN('F', 3))

typedef enum {
  LED_STATE_OFF,
  LED_STATE_RED,
  LED_STATE_BLUE,
  LED_STATE_GREEN,
  LED_STATE_YELLOW,
  LED_STATE_WHITE,
} LedState;

void enable_leds(void);
void set_led_state(LedState state);
