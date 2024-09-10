CC=arm-none-eabi-gcc
CFLAGS=-mcpu=cortex-m3 -mthumb -nostdlib
CPPFLAGS=-DSTM32F103x6 \
	 -Ivendor/CMSIS/Device/ST/STM32F1/Include \
	 -Ivendor/CMSIS/CMSIS/Core/Include

LINKER_FILE=linkscript.ld
LDFLAGS=-T $(LINKER_FILE)


all: blink2.elf

blink2.elf: main.o startup.o system_stm32f1xx.o
	$(CC) $(CFLAGS) $(CPPFLAGS) $(LDFLAGS) $^ -o blink2.elf

main.o: main.c
	$(CC) $(CFLAGS) $(CPPFLAGS) main.c -c

startup.o: startup.c
	$(CC) $(CFLAGS) $(CPPFLAGS) startup.c -c

system_stm32f1xx.o: vendor/CMSIS/Device/ST/STM32F1/Source/Templates/system_stm32f1xx.c
	$(CC) $(CFLAGS) $(CPPFLAGS) vendor/CMSIS/Device/ST/STM32F1/Source/Templates/system_stm32f1xx.c -c

.PHONY: clean
clean:
	rm -f *.o *.elf

PROGRAMMER=openocd
PROGRAMMER_FLAGS=-f interface/cmsis-dap.cfg -f target/stm32f1x.cfg

flash: blink2.elf
	$(PROGRAMMER) $(PROGRAMMER_FLAGS) -c "program blink2.elf verify reset exit"