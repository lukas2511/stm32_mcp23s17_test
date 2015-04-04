UNIXTIME ?= $(shell date +%s)

TARGET ?= test_bincount
LINKTHING ?= stm32.ld

all: $(TARGET).bin

%.o: %.c libopencm3/lib/stm32/f1/timer.o
	arm-none-eabi-gcc -Os -g -Wextra -Wshadow -Wimplicit-function-declaration -Wredundant-decls -Wmissing-prototypes -Wstrict-prototypes -fno-common -ffunction-sections -fdata-sections -MD -Wall -Wundef -I./libopencm3/include -DSTM32F1 -mthumb -mcpu=cortex-m3 -msoft-float -mfix-cortex-m3-ldrd -c $< -o $@

libopencm3/lib/stm32/f1/timer.o:
	git submodule init
	git submodule update
	make -C libopencm3 -j

%.elf: %.o
	arm-none-eabi-gcc --static -nostartfiles -L./libopencm3/lib -T$(LINKTHING) -Wl,--gc-sections -mthumb -mcpu=cortex-m3 -msoft-float -mfix-cortex-m3-ldrd $< -lopencm3_stm32f1 -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group -o $@

%.bin: %.elf
	arm-none-eabi-objcopy -Obinary $< $@

flash-%.bin: %.bin
	st-flash write $< 0x08000000

install: flash-$(TARGET).bin

clean:
	rm -f *.o *.bin *.elf *.d
