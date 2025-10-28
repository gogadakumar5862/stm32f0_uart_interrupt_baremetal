CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy

CFLAGS=-mcpu=cortex-m0 -mthumb -O0 -Wall -DSTM32F051x8 \
       -I./ \
	   -IDrivers/CMSIS/Include \
	   -IDrivers/CMSIS/Device/ST/STM32F0xx/Include 

#LDFLAGS=-Tstm32g474re.ld -nostdlib -Wl,--gc-sections,-Map=blinky.map
LDFLAGS=-Tstm32f051r8t6.ld -nostdlib -Wl,--gc-sections,-Map=uart_interrupt.map -lc -lgcc -lnosys
SRCS = src/main.c \
	   uart.c \
       startup_stm32f051r8t6.c \
	   Drivers/CMSIS/Device/ST/STM32F0xx/Source/Templates/system_stm32f0xx.c
OBJS = $(SRCS:.c=.o)

# Default target: build ELF + HEX + BIN
all: uart_interrupt.elf uart_interrupt.hex uart_interrupt.bin

# Link step
uart_interrupt.elf: $(OBJS)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

# Convert ELF -> HEX
uart_interrupt.hex: uart_interrupt.elf
	$(OBJCOPY) -O ihex $< $@

# Convert ELF -> BIN
uart_interrupt.bin: uart_interrupt.elf
	$(OBJCOPY) -O binary $< $@

# Compile C -> object
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# Clean up
clean:
	rm -f $(OBJS) uart_interrupt.elf uart_interrupt.hex uart_interrupt.bin uart_interrupt.map

