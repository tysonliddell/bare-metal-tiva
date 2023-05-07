#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BIT(x) ((uint32_t)1 << (x))
#define SYSTEM_CLOCK_FREQ_HZ_DEFAULT (16000000)
#define PIOSC_FREQ_HZ (16000000)
#define GPIO_UNLOCK_VALUE (0x4C4F434B)

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

void spin(volatile uint32_t count) {
  while (count--)
    (void)0;
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

static inline void clear_bit(volatile uint32_t *address, uint8_t bit_num) {
  *address &= ~(1U << bit_num);
}

static inline void set_bit(volatile uint32_t *address, uint8_t bit_num) {
  *address |= 1U << bit_num;
}

static inline bool get_bit(volatile uint32_t *address, uint8_t bit_num) {
  return *address & (1U << bit_num);
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

  float brd = SYSTEM_CLOCK_FREQ_HZ_DEFAULT / (16.f * (float)baud_rate);
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

int main(void) {
  systick_init();
  gpio_set_mode(GPIO_PIN('F', 1), GPIO_MODE_OUTPUT);
  gpio_set_mode(GPIO_PIN('F', 2), GPIO_MODE_OUTPUT);
  gpio_set_mode(GPIO_PIN('F', 3), GPIO_MODE_OUTPUT);

  uint8_t uart_num = 0;
  uart_init(uart_num, 115200);
  UARTMem *uart = UARTS[uart_num];

  uint32_t timer_expiry, period_ms = 500;
  uint32_t state = 0;
  for (;;) {
    if (timer_expired(&timer_expiry, period_ms)) {
      bool led_state = !(state & 0x1);
      uint32_t led_pin = state / 2 + 1;
      gpio_set_pin(GPIO_PIN('F', led_pin), led_state);
      state = state == 5 ? 0 : state + 1;

      uart_write_buf(uart, "hi\r\n", 4);
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
