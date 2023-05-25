#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BIT(x) ((uint32_t)1 << (x))
#define SYSTEM_CLOCK_FREQ_HZ_DEFAULT (16000000)
#define SYSTEM_CLOCK_FREQ_HZ_MAX (80000000)
#define PIOSC_FREQ_HZ (16000000)
#define GPIO_UNLOCK_VALUE (0x4C4F434B)
#define SYSTEM_CLOCK_FREQ_HZ SYSTEM_CLOCK_FREQ_HZ_MAX

typedef struct {
  volatile uint32_t DID0, DID1, RESERVED1[10], PBORCTL, RESERVED2[7], RIS, IMC,
      MISC, RESC, RCC, RESERVED3[2], GPIOHBCTL, RCC2, RESERVED4[2], MOSCCTL,
      RESERVED5[49], DSLPCLKCFG, RESERVED6[1], SYSPROP, PIOSCCAL, PIOSCSTAT,
      RESERVED7[2], PLLFREQ0, PLLFREQ1, PLLSTAT, RESERVED8[573];
} SYSCTLMem;
#define SYSCTL ((SYSCTLMem *)0x400FE000)

typedef struct {
  volatile uint32_t GPIODATA_MASKS[255], GPIODATA, GPIODIR, GPIOIS, GPIOIBE,
      GPIOIEV, GPIOIM, GPIORIS, GPIOMIS, GPIOICR, GPIOAFSEL, RESERVED1[62],
      GPIODEN, GPIOLOCK, GPIOCR, GPIOAMSEL, GPIOPCTL, GPIOADCCTL, GPIODMACTL;
} GPIOMem;
#define GPIOMEM_A ((GPIOMem *)0x40004000)
#define GPIOMEM_B ((GPIOMem *)0x40005000)
#define GPIOMEM_C ((GPIOMem *)0x40006000)
#define GPIOMEM_D ((GPIOMem *)0x40007000)
#define GPIOMEM_E ((GPIOMem *)0x40024000)
#define GPIOMEM_F ((GPIOMem *)0x40025000)

static GPIOMem *GPIO_PORTS[] = {
    GPIOMEM_A, GPIOMEM_B, GPIOMEM_C, GPIOMEM_D, GPIOMEM_E, GPIOMEM_F,
};

typedef struct {
  uint8_t port;
  uint8_t pin;
} GPIOPin;
#define GPIO_PIN(port, pin) ((GPIOPin){(port - 'A'), ((uint8_t)(pin))})

typedef struct {
  volatile uint32_t RCGCWD, RCGCTIMER, RCGCGPIO, RCGCDMA, RESERVED1, RCGCHIB,
      RCGCUART, RCGCSSI, RCGCI2C, RESERVED2, RCGCUSB, RESERVED3[2], RCGCCAN,
      RCGCADC, RCGCACMP, RCGCPWM, RCGCQEI, RESERVED4[4], RCGCEEPROM, RCGCWTIMER;
} RCGCMem;
#define RCGC ((RCGCMem *)0x400FE600)

typedef enum { GPIO_MODE_INPUT, GPIO_MODE_OUTPUT } GPIOMode;

typedef struct {
  volatile uint32_t STCTRL, STRELOAD, STCURRENT;
} SysTickMem;
#define SYSTICK ((SysTickMem *)0xE000E010)

typedef struct {
  volatile uint32_t UARTDR, UARTRSR_UARTECR, RESERVED1[4], UARTFR, RESERVED2[1],
      UARTILPR, UARTIBRD, UARTFBRD, UARTLCRH, UARTCTL, UARTIFLS, UARTIM,
      UARTRIS, UARTMIS, UARTICR, UARTDMACTL, RESERVED3[22], UART9BITADDR,
      UART9BITAMASK, RESERVED4[965], UARTPP, RESERVED5[1], UARTCC;
} UARTMem;
#define UART0 ((UARTMem *)0x4000C000)
#define UART1 ((UARTMem *)0x4000D000)
#define UART2 ((UARTMem *)0x4000E000)
#define UART3 ((UARTMem *)0x4000F000)
#define UART4 ((UARTMem *)0x40010FFF)
#define UART5 ((UARTMem *)0x40011FFF)
#define UART6 ((UARTMem *)0x40012FFF)
#define UART7 ((UARTMem *)0x40013FFF)

static UARTMem *UARTS[] = {
    UART0, UART1, UART2, UART3, UART4, UART5, UART6, UART7,
};

typedef struct {
  volatile uint32_t GPTMCFG, GPTMTAMR, GPTMTBMR, GPTMCTL, GPTMSYNC, RESERVED1,
      GPTMIMR, GPTMRIS, GPTMMIS, GPTMICR, GPTMTAILR, GPTMTBILR, GPTMTAMATCHR,
      GPTMTBMATCHR, GPTMTAPR, GPTMTBPR, GPTMTAPMR, GPTMTBPMR, GPTMTAR, GPTMTBR,
      GPTMTAV, GPTMTBV, GPTMRTCPD, GPTMTAPS, GPTMTBPS, GPTMTAPV, GPTMTBPV,
      RESERVED2[981], GPTMPP;
} GPTMMem;
#define TIMER0 ((GPTMMem *)0x40030000)
#define TIMER1 ((GPTMMem *)0x40031000)
#define TIMER2 ((GPTMMem *)0x40032000)
#define TIMER3 ((GPTMMem *)0x40033000)
#define TIMER4 ((GPTMMem *)0x40034000)
#define TIMER5 ((GPTMMem *)0x40035000)
#define WIDETIMER0 ((GPTMMem *)0x40036000)
#define WIDETIMER1 ((GPTMMem *)0x40037000)
#define WIDETIMER2 ((GPTMMem *)0x4004C000)
#define WIDETIMER3 ((GPTMMem *)0x4004D000)
#define WIDETIMER4 ((GPTMMem *)0x4004E000)
#define WIDETIMER5 ((GPTMMem *)0x4004F000)

static GPTMMem *TIMERS[] = {
    TIMER0, TIMER1, TIMER2, TIMER3, TIMER4, TIMER5,
};

static inline void spin(volatile uint32_t count) {
  while (count--)
    (void)0;
}

static inline void clear_bit(volatile uint32_t *address, uint8_t bit_num) {
  *address &= ~(1u << bit_num);
}

static inline void set_bit(volatile uint32_t *address, uint8_t bit_num) {
  *address |= 1u << bit_num;
}

static inline bool get_bit(volatile uint32_t *address, uint8_t bit_num) {
  return *address & (1u << bit_num);
}

static inline void setup_80mhz_pll_clock(void) {
  // bypass the PLL while configuring it
  set_bit(&SYSCTL->RCC, 11);
  set_bit(&SYSCTL->RCC2, 11);

  // RCC1 setup
  // set XTAL to encoding for 16.0 MHz external main oscillator crystal
  SYSCTL->RCC &= ~(0x1Fu << 6);
  SYSCTL->RCC |= (0x15 << 6);

  set_bit(&SYSCTL->RCC, 22); // enable system clock divider (PLL requires this)

  // RCC2 setup (must come after RCC1 setup)
  set_bit(&SYSCTL->RCC2, 31); // use RCC2 (to access DIV400 and 80 MHz clock)

  // set the oscillator source to MOSC encoding (0x0)
  SYSCTL->RCC2 &= ~(0x3u << 4);

  // power up the PLL by clearing the PWRDN bit
  clear_bit(&SYSCTL->RCC2, 13);

  set_bit(&SYSCTL->RCC2, 30); // use DIV400
  SYSCTL->RCC2 &= ~(0x7Fu << 22);
  SYSCTL->RCC2 |= (0x04 << 22); // divide 400 MHz PLL by 4+1 -> 80 MHz

  // while (get_bit(&SYSCTL->PLLSTAT, 0) == 0) {
  while (get_bit(&SYSCTL->RIS, 6) == 0) {
    // wait for PLL to reconverge/lock
    spin(1);
  }

  // enable the PLL by clearing the BYPASS bit
  clear_bit(&SYSCTL->RCC2, 11);
}

static inline void gpio_set_mode(GPIOPin gpiopin, GPIOMode mode) {
  RCGC->RCGCGPIO |= 1 << gpiopin.port; // ensure clock is enabled for GPIO port
  spin(3);

  GPIOMem *gpio = GPIO_PORTS[gpiopin.port];
  gpio->GPIODIR &= ~((uint32_t)1 << gpiopin.pin);
  gpio->GPIODIR |= mode << gpiopin.pin;
  gpio->GPIODEN |= 1 << gpiopin.pin;
}

static inline void gpio_set_pin(GPIOPin gpiopin, bool set) {
  GPIOMem *gpio = GPIO_PORTS[gpiopin.port];
  int mask =
      1 << gpiopin.pin; // Since GPIODATA_MASKS is an array of uint32, adding
                        // the mask adds `mask * 4`, i.e. `mask << 2` to the
                        // memory address, which is what the datasheet requires.
  *(gpio->GPIODATA_MASKS + mask) = set ? 0xFF : 0x00;
}

static inline void systick_init() {
  uint32_t systick_freq = SYSTEM_CLOCK_FREQ_HZ;
  uint32_t ticks_per_ms = systick_freq / 1000 - 1;
  SYSTICK->STRELOAD = ticks_per_ms;
  SYSTICK->STCURRENT = 0x01;                  // clear the STCURRENT REGISTER
  SYSTICK->STCTRL = BIT(0) | BIT(1) | BIT(2); // enable systick, timer interrupt
                                              // and use system clock
}

static inline bool timer_expired(uint32_t *expiry, uint32_t period_ms,
                                 uint32_t now) {
  if (now + period_ms < *expiry)
    *expiry = 0; // Timer wrapped? Reset it.
  if (*expiry == 0)
    *expiry = now + period_ms; // First poll or timer wrapped? Set expiration

  if (*expiry > now)
    return false;

  *expiry = (now - *expiry) > period_ms ? now + period_ms : *expiry + period_ms;
  return true;
}

static inline void gpio_enable_af(GPIOPin gpiopin, uint8_t af) {
  GPIOMem *gpio = GPIO_PORTS[gpiopin.port];

  set_bit(&RCGC->RCGCGPIO, gpiopin.port); // make sure GPIO module has a clock
  spin(3);
  bool is_gpio_locked = gpio->GPIOLOCK == 0x01 ? true : false;
  uint32_t original_cr = gpio->GPIOCR;
  if (is_gpio_locked) {
    gpio->GPIOLOCK = GPIO_UNLOCK_VALUE;
    set_bit(&gpio->GPIOCR, gpiopin.pin);
  }
  set_bit(&gpio->GPIOAFSEL, gpiopin.pin); // use alternate function

  // select alternate function
  gpio->GPIOPCTL &= ~(0xFUL) << (4 * gpiopin.pin);
  gpio->GPIOPCTL |= af << (4 * gpiopin.pin);

  set_bit(&gpio->GPIODEN, gpiopin.pin); // make sure gpio pin is enabled

  if (is_gpio_locked) {
    gpio->GPIOCR = original_cr;
    gpio->GPIOLOCK = 1;
  }
}

static inline void uart_init(uint8_t uart_num, uint32_t baud_rate) {
  GPIOPin rx, tx;

  switch (uart_num) {
  case 0:
    rx = GPIO_PIN('A', 0);
    tx = GPIO_PIN('A', 1);
    break;
  case 1:
    rx = GPIO_PIN('B', 0);
    tx = GPIO_PIN('B', 1);
    break;
  case 2:
    rx = GPIO_PIN('D', 6);
    tx = GPIO_PIN('D', 7);
    break;
  case 3:
    rx = GPIO_PIN('C', 7);
    tx = GPIO_PIN('C', 7);
    break;
  case 4:
    rx = GPIO_PIN('C', 4);
    tx = GPIO_PIN('C', 5);
    break;
  case 5:
    rx = GPIO_PIN('E', 4);
    tx = GPIO_PIN('E', 5);
    break;
  case 6:
    rx = GPIO_PIN('D', 4);
    tx = GPIO_PIN('D', 5);
    break;
  case 7:
    rx = GPIO_PIN('E', 0);
    tx = GPIO_PIN('E', 1);
    break;
  }

  uint8_t uart_af = 1;
  set_bit(&RCGC->RCGCUART, uart_num); // enable UART clock
  spin(3);
  gpio_enable_af(rx, uart_af);
  gpio_enable_af(tx, uart_af);

  float brd = SYSTEM_CLOCK_FREQ_HZ / (16.f * (float)baud_rate);
  uint32_t brd_int = (uint32_t)brd;
  uint32_t brd_frac = (uint32_t)((brd - (float)brd_int) * 64.0f + 0.5f);

  UARTMem *uart = UARTS[uart_num];
  clear_bit(&uart->UARTCTL, 0); // disable uart before configuring
  clear_bit(&uart->UARTCTL, 5); // make sure HSE (high-speed enable) if off
  uart->UARTCC = 0x00;          // use system clock source for baud clock
  uart->UARTIBRD = brd_int;
  uart->UARTFBRD = brd_frac;
  uart->UARTLCRH = 0x00000060; // set data length to 8 bits
  set_bit(&uart->UARTCTL, 0);  // enable uart after configuring
}

static inline void uart_write_byte(UARTMem *uart, uint8_t byte) {
  uart->UARTDR = byte;
  while (get_bit(&uart->UARTFR, 5))
    spin(1);
}

static inline void uart_write_buf(UARTMem *uart, char *buf, size_t len) {
  while (len-- > 0)
    uart_write_byte(uart, (uint8_t)*buf++);
}

// static inline void periodic_timer_init(GPTMMem *timer) {
static inline void periodic_timer_init(uint8_t timer_num) {
  GPTMMem *timer = TIMERS[timer_num];

  // Give the timer module a clock
  set_bit(&RCGC->RCGCTIMER, timer_num);

  // Set up the timer to count up and wrap around periodically.
  clear_bit(&timer->GPTMCTL, 0); // disable TIMER A
  timer->GPTMCFG = 0; // concatenate TIMER A and TIMER B into a single timer

  timer->GPTMTAMR = 0;
  set_bit(&timer->GPTMTAMR, 4); // count upwards
  timer->GPTMTAMR |= 0x2;       // periodic timer mode

  timer->GPTMTAILR =
      0xFFFFFFFF; // value to count up to before starting back at zero

  set_bit(&timer->GPTMCTL, 0); // enable TIMER A and start counting
}

static inline uint32_t get_timer_value(GPTMMem *timer) {
  return timer->GPTMTAV; // value of the free-running timer
}
