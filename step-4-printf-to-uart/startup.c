extern int main(void); // defined in main.c

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

extern void SysTick_Handler(void); // defined in main.c
extern void _estack(void); // defined in link.ld

// 16 standard and 139 TM4C123GH6PM-specific interrupt handlers
__attribute__((section(".vectors"))) void (*const tab[16 + 139])(void) = {
    _estack, _reset, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, SysTick_Handler};
