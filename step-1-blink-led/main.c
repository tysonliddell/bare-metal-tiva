#include <stdbool.h>
#include <stdint.h>

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

void enable_gpio_pin_digital(GPIOMem *gpio, GPIOPin pin) {
  gpio->GPIODEN |= 1 << pin;
}

void enable_gpio_port_clock(char port) {
  RCGC->RCGCGPIO |= 1 << (port - 'A');
  // After enabling the clock for a midule we are supposed to wait 3 system
  // clock cycles before accessing registers for the module.
  spin(100);
}

int main(void) {
  enable_gpio_port_clock('F');
  enable_gpio_pin_digital(GPIOF, GPIO_PIN1);
  enable_gpio_pin_digital(GPIOF, GPIO_PIN2);
  enable_gpio_pin_digital(GPIOF, GPIO_PIN3);
  gpio_set_mode(GPIOF, GPIO_PIN1, GPIO_MODE_OUTPUT);
  gpio_set_mode(GPIOF, GPIO_PIN2, GPIO_MODE_OUTPUT);
  gpio_set_mode(GPIOF, GPIO_PIN3, GPIO_MODE_OUTPUT);

  for (;;) {
    gpio_set_pin(GPIOF, GPIO_PIN1, true);
    spin(999999);
    gpio_set_pin(GPIOF, GPIO_PIN1, false);
    spin(999999);
    gpio_set_pin(GPIOF, GPIO_PIN2, true);
    spin(999999);
    gpio_set_pin(GPIOF, GPIO_PIN2, false);
    spin(999999);
    gpio_set_pin(GPIOF, GPIO_PIN3, true);
    spin(999999);
    gpio_set_pin(GPIOF, GPIO_PIN3, false);
    spin(999999);
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
__attribute__((section(".vectors"))) void (*tab[16 + 139])(void) = {_estack,
                                                                    _reset};
