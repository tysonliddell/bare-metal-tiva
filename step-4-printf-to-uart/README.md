# Redirecting `printf` to UART
## Troubleshooting
### Fixing an old linker warning
Since starting the exercises in this repo the following linker error (which I have been ignoring) has occurred:

```
warning: firmware.elf has a LOAD segment with RWX permissions
```

While debugging another issue I wanted to fix this warning, just in case it was causing the problem. Inspecting the section headers of the compiled object file shows that the interrupt vector table is the LOAD segment with write permission:
```
$ arm-none-eabi-objdump -h firmware.elf
firmware.elf:     file format elf32-littlearm
Sections:
Idx Name          Size      VMA       LMA       File off  Algn
  0 .vectors      0000026c  00000000  00000000  00001000  2**2
                  CONTENTS, ALLOC, LOAD, DATA
  1 .text         00000bd4  0000026c  0000026c  0000126c  2**2
                  CONTENTS, ALLOC, LOAD, READONLY, CODE
  2 .rodata       00000024  00000e40  00000e40  00001e40  2**2
                  CONTENTS, ALLOC, LOAD, READONLY, DATA
  ...
  <SNIP>
```

This was fixed by changing the declaration of the vector table in the startup code from an array of pointers to functions to an array of `const` pointers to functions like so:
```diff
// in startup.c

-__attribute__((section(".vectors"))) void (*tab[16 + 139])(void) = {
+__attribute__((section(".vectors"))) void (*const tab[16 + 139])(void) = {
     _estack, _reset, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, SysTick_Handler};
```

Making this change stops the linker warning and the `vectors` section of the object file is now read-only, as it should be:
```
$ arm-none-eabi-objdump -h firmware.elf
firmware.elf:     file format elf32-littlearm
Sections:
Idx Name          Size      VMA       LMA       File off  Algn
  0 .vectors      0000026c  00000000  00000000  00001000  2**2
                  CONTENTS, ALLOC, LOAD, READONLY, DATA
  1 .text         00000bd4  0000026c  0000026c  0000126c  2**2
                  CONTENTS, ALLOC, LOAD, READONLY, CODE
  2 .rodata       00000024  00000e40  00000e40  00001e40  2**2
                  CONTENTS, ALLOC, LOAD, READONLY, DATA
```

### Call to `printf` sending an empty string to `_write` syscall
Following the guide as written to implement IO retargeting to the UART did not work. The `_write` syscall would receive an empty string on every call to `printf`. Stepping through the newlib-nano assembly code in `gdb` indicated that the string `foo` in a call to `printf("foo")` was being written to a temporary buffer somewhere (inside a `struct _reent`). This seems to match the source code to newlib [here](https://github.com/bminor/newlib/blob/8144619bad0c25b1a27edbd0085664ac338822af/newlib/libc/stdio/puts.c#L110) and [here](https://github.com/bminor/newlib/blob/8144619bad0c25b1a27edbd0085664ac338822af/newlib/libc/include/stdio.h#L697) - I couldn't find the source code for newlib-nano. Unfortunately, the buffer that was being written to was located in the vector table, somewhere that shouldn't be written to at runtime. I stumbled upon [this blog post][heap-info] that mentioned that newlib's `printf` uses `malloc` put data on the heap. The `_sbrk` syscall needs to be implemented for malloc to work, as mentioned in the *Heap space* section of this [linker script deep dive][thoroughly-commented-linker-script]. Looking at the solution for this exercise, I found that the author included an `_sbrk` implementation in `syscalls.c`, even though this wasn't mentioned in the instructions:

```
void *_sbrk(int incr) {
  extern char _end;
  static unsigned char *heap = NULL;
  unsigned char *prev_heap;
  if (heap == NULL) heap = (unsigned char *) &_end;
  prev_heap = heap;
  heap += incr;
  return prev_heap;
}
```

Replacing the initial stub with this implementation fixed the issues and caused the `printf` statements to send data across the UART.


## Useful links
1. [Running gdb in TUI mode](https://stackoverflow.com/a/2422063/6534028)

[user-guide]: /docs/tiva-c-launchpad-user-guide.pdf "Tiva C Launchpad User Guide"
[datasheet]: /docs/tm4c123gh6pm-datasheet.pdf "TM4C123GH6PM Data Sheet"
[heap-info]: https://sushihangover.github.io/arm-cortex-m3-bare-metal-with-newlib/ "Cortex-M3 Bare-metal with NEWLIB"
[thoroughly-commented-linker-script]: https://blog.thea.codes/the-most-thoroughly-commented-linker-script/ "The most thoroughly commented linker script (probably)"

