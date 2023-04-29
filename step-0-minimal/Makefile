CFLAGS := -W -Wall -Wextra -Werror -Wundef -Wshadow -Wdouble-promotion \
            -Wformat-truncation -fno-common -Wconversion \
            -g3 -Os -ffunction-sections -fdata-sections -I. \
            -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 $(EXTRA_CFLAGS)
LINKSCRIPT := link.ld
LDFLAGS := -T$(LINKSCRIPT) -nostartfiles -nostdlib --specs nano.specs -lc -lgcc -Wl,--gc-sections -Wl,-Map=$@.map
SOURCES := main.c

.PHONY: build
build: firmware.elf

firmware.elf: $(SOURCES) $(LINKSCRIPT)
	arm-none-eabi-gcc $(SOURCES) $(CFLAGS) $(LDFLAGS) -o $@

firmware.bin: firmware.elf
	arm-none-eabi-objcopy -O binary $< $@

.PHONY: flash
flash: firmware.bin
	openocd -f board/ek-tm4c123gxl.cfg -c "program ./firmware.bin reset exit"

.PHONY: clean
clean:
	rm firmware.*
