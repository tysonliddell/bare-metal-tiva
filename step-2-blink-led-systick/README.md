# Using Arm SysTick
The Arm SysTick timer can be used to better control the timing of the blinking LED.

## Using SysTick interrupt for precise timing
The [data sheet][datasheet] shows that system clock frequency of the TM4C123GH6PM can be configured using the `SYSDIV` field of the `RCC` register. By default the clock frequency will be 12.5 MHz if nothing is changed. For simplicity, let's leave the system clock frequency at the default value.

Generating timing measure in milliseconds can then be achieved as follows:
1. Initialise SysTick to fire an interrupt to fire every `12500000 / 1000` cycles. The data sheet says that to achieve this the `STRELOAD` register needs to be set to `12500000 / 1000 - 1`.
2. Have the interrupt increment some static value each time the interrupt runs.
3. The static value will then increment once per millisecond.

## Increasing clock accuracy with PIOSC
The system clock (`SysClk`) was used as the clock source for the SysTick timer by setting the appropriate bit in the `STCTRL` register. When comparing the LED flashing once per second to a stopwatch, I found that this wasn't very accurate. Sometimes I observed 8-9 blinks in a 10 second window. Switching to the precision internal oscillator (`PIOSC`) as a clock source (i.e. zeroing the `CLK_SRC` bit in `STCTRL`) proved to be much more accurate.

## Misc
### Inspecting generated machine code
The machine code generated for the `delay` function can be inspected using `objdump` to disassemble the compiled code as follows:
```
$ arm-none-eabi-objdump -d firmware.elf
firmware.elf:     file format elf32-littlearm

... SNIP
00000290 <delay>:
 290:   4a03            ldr     r2, [pc, #12]   @ (2a0 <delay+0x10>)
 292:   6813            ldr     r3, [r2, #0]
 294:   4403            add     r3, r0
 296:   6811            ldr     r1, [r2, #0]
 298:   4299            cmp     r1, r3
 29a:   d3fc            bcc.n   296 <delay+0x6>
 29c:   4770            bx      lr
 29e:   bf00            nop
 2a0:   20000000        .word   0x20000000
... SNIP
```

And here what is generated if we don't include the `volatile` keyword for the `uint32_t until` variable in the `delay` function:
```
00000290 <delay>:
 290:   4b02            ldr     r3, [pc, #8]    @ (29c <delay+0xc>)
 292:   681b            ldr     r3, [r3, #0]
 294:   4418            add     r0, r3
 296:   4283            cmp     r3, r0
 298:   d3fd            bcc.n   296 <delay+0x6>
 29a:   4770            bx      lr
 29c:   20000000        .word   0x20000000
```
Note that the `s_ticks` value (`r3`) is never updated, and the loop at addresses `0x296 - 0x298` will run forever. In the earlier disassembly, where `volatile` was used, the `s_ticks` value (`r1`) is updated on each iteration of the loop.

[user-guide]: /docs/tiva-c-launchpad-user-guide.pdf "Tiva C Launchpad User Guide"
[datasheet]: /docs/tm4c123gh6pm-datasheet.pdf "TM4C123GH6PM Data Sheet"
