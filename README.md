# Bare metal Tiva C Launchpad
Adapting https://github.com/cpq/bare-metal-programming-guide to build some bare metal projects on a Tiva C launchpad.

## Why do this?
To use the tools I am familiar with (make, gcc, etc.) to build and flash some simple programs for the microcontroller without an IDE like IAR or CCS doing any magic. I struggled trying to do this at first and decided to double down and understand:
- how my embedded programs are compiled,
- how they are sent to the appropriate section of flash memory on the MCU, and
- how the (on-chip) debugger works.

## Sections
0. [Setting up a minimal dev environment](./step-0-minimal/README.md)
1. [Making the LED blink](./step-1-blink-led/README.md)
