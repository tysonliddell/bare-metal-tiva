#include <stdbool.h>
#include <stdint.h>

#define BIT(x) ((uint32_t)1 << (x))
#define SYSTEM_CLOCK_FREQ_HZ_DEFAULT (12500000)
#define PIOSC_FREQ_HZ (16000000)

typedef struct {
  volatile uint32_t GPIODATA_MASKS[255], GPIODATA, GPIODIR, RESERVED1[70],
      GPIODEN, RESERVED2[1024 - 255 - 1 - 1 - 70 - 1];
} GPIOMem;
#define GPIOA ((GPIOMem *)0x40004000)
#define GPIOB ((GPIOMem *)0x40005000)
#define GPIOC ((GPIOMem *)0x40006000)
#define GPIOD ((GPIOMem *)0x40007000)
#define GPIOE ((GPIOMem *)0x40024000)
#define GPIOF ((GPIOMem *)0x40025000)

typedef struct {
  volatile uint32_t RCGCWD, RCGCTIMER, RCGCGPIO, RCGCDMA, RESERVED1, RCGCHIB,
      RCGCUART, RCGCSSI, RCGCI2C, RESERVED2, RCGCUSB, RESERVED3[2], RCGCCAN,
      RCGCADC, RCGCACMP, RCGCPWM, RCGCQEI, RESERVED4[4], RCGCEEPROM, RCGCWTIMER;
} RCGCMem;
#define RCGC ((RCGCMem *)0x400FE600)

typedef enum { GPIO_MODE_INPUT, GPIO_MODE_OUTPUT } GPIOMode;
typedef enum {
  GPIO_PIN0,
  GPIO_PIN1,
  GPIO_PIN2,
  GPIO_PIN3,
  GPIO_PIN4,
  GPIO_PIN5,
  GPIO_PIN6,
  GPIO_PIN7,
} GPIOPin;

typedef struct {
  volatile uint32_t STCTRL, STRELOAD, STCURRENT;
} SysTickMem;
#define SYSTICK ((SysTickMem *)0xE000E010)

void spin(volatile uint32_t count) {
  while (count--)
    (void)0;
}

static inline void gpio_set_mode(GPIOMem *gpio, GPIOPin pin, GPIOMode mode) {
  gpio->GPIODIR &= ~((uint32_t)1 << pin);
  gpio->GPIODIR |= mode << pin;
}

static inline void gpio_set_pin(GPIOMem *gpio, GPIOPin pin, bool set) {
  int mask = 1 << pin; // Since GPIODATA_MASKS is an array of uint32, adding the
                       // mask adds `mask * 4`, i.e. `mask << 2` to the memory
                       // address, which is what the datasheet requires.
  *(gpio->GPIODATA_MASKS + mask) = set ? 0xFF : 0x00;
}

static inline void enable_gpio_pin_digital(GPIOMem *gpio, GPIOPin pin) {
  gpio->GPIODEN |= 1 << pin;
}

static inline void enable_gpio_port_clock(char port) {
  RCGC->RCGCGPIO |= 1 << (port - 'A');
  // After enabling the clock for a midule we are supposed to wait 3 system
  // clock cycles before accessing registers for the module.
  spin(100);
}

static inline void systick_init() {
  uint32_t systick_freq = PIOSC_FREQ_HZ / 4;
  uint32_t ticks_per_ms = systick_freq / 1000 - 1;
  SYSTICK->STRELOAD = ticks_per_ms;
  SYSTICK->STCURRENT = 0x01;         // clear the STCURRENT REGISTER
  SYSTICK->STCTRL = BIT(0) | BIT(1); // enable systick, timer interrupt and use
                                     // precision internal oscillator (PIOSC)
}

static volatile uint32_t s_ticks;
void SysTick_Handler(void) { s_ticks++; }

void delay(unsigned ms) {
  uint32_t until = s_ticks + ms;
  while (s_ticks < until)
    (void)0;
}

bool timer_expired(uint32_t *expiry, uint32_t period_ms) {
  uint32_t now = s_ticks;
  if (now + period_ms < *expiry)
    *expiry = 0; // Timer wrapped? Reset it.
  if (*expiry == 0)
    *expiry = now + period_ms; // First poll or timer wrapped? Set expiration

  if (*expiry > now)
    return false;

  *expiry = (now - *expiry) > period_ms ? now + period_ms : *expiry + period_ms;
  return true;
}

int main(void) {
  systick_init();
  enable_gpio_port_clock('F');
  enable_gpio_pin_digital(GPIOF, GPIO_PIN1);
  enable_gpio_pin_digital(GPIOF, GPIO_PIN2);
  enable_gpio_pin_digital(GPIOF, GPIO_PIN3);
  gpio_set_mode(GPIOF, GPIO_PIN1, GPIO_MODE_OUTPUT);
  gpio_set_mode(GPIOF, GPIO_PIN2, GPIO_MODE_OUTPUT);
  gpio_set_mode(GPIOF, GPIO_PIN3, GPIO_MODE_OUTPUT);

  uint32_t timer_expiry, period_ms = 500;
  uint32_t state = 0;
  for (;;) {
    if (timer_expired(&timer_expiry, period_ms)) {
      bool led_state = !(state & 0x1);
      uint32_t led_pin = state / 2 + 1;
      gpio_set_pin(GPIOF, led_pin, led_state);
      state = state == 5 ? 0 : state + 1;
    }
  }
  return 0;
}

// startup code
__attribute__((naked, noreturn)) void _reset(void) {
  // memset .bss to zero and copy .data section to RAM
  extern long _sbss, _ebss, _sdata, _edata, _sidata;
  for (long *dst = &_sbss; dst < &_ebss; dst++)
    *dst = 0;
  for (long *dst = &_sdata, *src = &_sidata; dst < &_edata;)
    *dst++ = *src++;

  main();
  for (;;)
    (void)0;
}

extern void _estack(void); // defined in link.ld

// 16 standard and 139 TM4C123GH6PM-specific interrupt handlers
__attribute__((section(".vectors"))) void (*tab[16 + 139])(void) = {
    _estack, _reset, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, SysTick_Handler};
