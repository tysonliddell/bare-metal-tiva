int main(void) {
    int i = 0;
    while (1) {
        i += 1;
    }
    return 0;
}

// startup code
__attribute__((naked, noreturn)) void _reset(void) {
    // memset .bss to zero and copy .data section to RAM
    extern long _sbss, _ebss, _sdata, _edata, _sidata;
    for (long *dst = &_sbss; dst < &_ebss; dst++) *dst = 0;
    for (long *dst = &_sdata, *src = &_sidata; dst < &_edata;) *dst++ = *src++;

    main();
    for (;;) (void) 0;
}

extern void _estack(void);  // defined in link.ld

// 16 standard and 139 TM4C123GH6PM-specific interrupt handlers
__attribute__((section(".vectors"))) void (*tab[16+139])(void) = {
    _estack, _reset
};
