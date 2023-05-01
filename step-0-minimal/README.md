# Setting up a minimal dev environment
## Memory layout
According to the [datasheet][datasheet], memory for the TM4C123GH6PM MCU that we are interested in is organised as follows:

|Region|Address Range            |Size  |
|------|-------------------------|------|
|Flash |0x0000.0000 - 0x0003.FFFF|256 KB|
|SRAM  |0x2000.0000 - 0x2000.7FFF|32 KB |

Memory is byte-addressable, 32-bit word aligned. This means that individual bytes can be read/written to using the address of the byte, but you cannot write a whole 32 bit word to an address that doesn't align with the start of a word (`0x0000.0000`, `0x0000.0004`, `0x0000.0008`, etc.).

### Flash
As seen in the 8.1 memory block diagram, the flash memory array is accessed through the flash memory controller. A quick read through the datasheet shows that the controller controls how the actual bits are read/written from/to the flash array. We shouldn't need to concern ourselves with these low-level details. Flashing the program will take care of the implementation details of writing to the flash array and program execution will read instructions from the flash array using memory map in the table above.

### SRAM
The SRAM is *bit-banded*, meaning that there is section of memory addresses that alias to individual bits in the SRAM. Normally, manipulating a single bit in a word involves multiple AND/OR/NOT instructions. However, we can read/write an individual bit in a single operation by reading/writing to a word in the bit-band alias region. This region is obviously a lot larger because it needs an address for each bit. The bit-band alias region for the SRAM is `0x2200.0000 - 0x220F.FFFF`. Bit-banding can be enabled with the `--bitband` compiler flag or the bit-band alias region can be accessed directly by the program.

*Note: bit-banding is an Arm Cortex-M3/M4 feature only. It's not available on other microcontrollers, so not many people use it.*

## GPIO
There are 6 GPIO ports on the TM4C123GH6PM: port A - port F. A GPIO Direction (`GPIODIR`) register for each port is used to put individual pins in input or output mode. The data is read from/written to a port using its `GPIODATA` register. GPIO ports (modules) are disabled be default to save power. A port is enabled using the `RCGCGPIO` register.

### Reading/setting GPIO bits using the address bus
To get around the read-modify-write instructions normally needed to read/write individual bits on the `GPIODATA` register, bits `[9:2]` of the address offset are used as a bit mask. For example, writing to address `GPIODATA + 0x098` will update the `0x98 >> 2` bits (`0010 0110`) in the `GPIODATA` register. 

### GPIO mode
By default the GPIO pins are set to *software control*, where we can read/write the pins from our program using the `GPIODATA` register as described above. *Hardware control* is also available.

## Compiling and flashing
- Compile a program for the MCU with `arm-none-eabi-gcc -mcpu=cortex-m4 main.c -c`.
- Create the appropriate linker script to map the interrupt vectors and text/data/bss sections to the appropriate addresses in the flash memory/SRAM. The linker script also needs to expose the start address for the MCU stack pointer in SRAM.
- Make sure that the reset interrupt routine in our program wipes the bss section in SRAM and copies the data section from flash to SRAM, and that the correct value for the initial MCU stack pointer is set using the value computed in the linker script.
- Generate the complete firmware with `arm-none-eabi-gcc -T link.ld -nostdlib main.o -o firmware.elf`.
- Since the ELF file is not a format that can be used on the MCU (ELF contains extra metadata, debug symbols, etc.), we need to turn it into the raw bytes needed for flashing. That is, the byte-for-byte binary code that will sit on the flash memory starting at address `0x0000.0000` is needed. To do this use `arm-none-eabi-objcopy -O binary firmware.elf firmware.bin`.
- Flash using `openocd -f board/ek-tm4c123gxl.cfg -c "program ./firmware.bin reset exit"`.

## Debugging with OpenOCD and gdb
The Tiva C launchpad has a built in in-circuit debugger for flashing and debugging the MCU. We can connect to it using OpenOCD:
```
openocd -f board/ek-tm4c123gxl.cfg
```
This will listen on port `3333` for gdb connections. The compiled ELF file (compiled with debug symbols) can then be used with `arm-gdb` to connect to OpenOCD and debug the program:
```
$ arm-none-eabi-gdb firmware.elf
GNU gdb (Arm GNU Toolchain 12.2 (Build arm-12-mpacbti.34)) 13.1.90.20230307-git
Copyright (C) 2023 Free Software Foundation, Inc.
License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>
This is free software: you are free to change and redistribute it.
There is NO WARRANTY, to the extent permitted by law.
Type "show copying" and "show warranty" for details.
This GDB was configured as "--host=aarch64-apple-darwin20.6.0 --target=arm-none-eabi".
Type "show configuration" for configuration details.
For bug reporting instructions, please see:
<https://bugs.linaro.org/>.
Find the GDB manual and other documentation resources online at:
    <http://www.gnu.org/software/gdb/documentation/>.

For help, type "help".
Type "apropos word" to search for commands related to "word"...
Reading symbols from firmware.elf...
(gdb) target remote localhost:3333
Remote debugging using localhost:3333
0x000002a8 in _reset () at main.c:50
50          for (;;) (void) 0;
(gdb) break main.c:45
Breakpoint 1 at 0x27a: file main.c, line 46.
Note: automatically using hardware breakpoints for read-only addresses.
(gdb) continue
```

[datasheet]: /datasheets/tm4c123gh6pm-datasheet.pdf "TM4C123GH6PM Data Sheet"
