CC = aarch64-none-elf-gcc
AS = aarch64-none-elf-as
LD = aarch64-none-elf-ld
OBJCOPY = aarch64-none-elf-objcopy

CFLAGS = -Wall -Wextra -O2 -ffreestanding -nostdlib -mcpu=cortex-a53
ASFLAGS = -g
LDFLAGS = -T src/linker.ld --nmagic

SRCS_C = src/core/kernel.c
SRCS_S = src/boot/boot.s
OBJS = $(SRCS_C:.c=.o) $(SRCS_S:.s=.o)

TARGET = ergenon-kernel.bin

.PHONY: all clean flash

all: $(TARGET)

$(TARGET): $(OBJS)
	$(LD) $(LDFLAGS) -o $@.elf $^
	$(OBJCOPY) -O binary $@.elf $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.s
	$(AS) $(ASFLAGS) $< -o $@

clean:
	rm -f *.o *.elf *.bin

flash:
	@echo "Flashing to target hardware not implemented yet."
	@echo "Use: openocd -f interface/stlink.cfg -f target/stm32h7x.cfg -c 'program ergenon-kernel.bin verify reset exit'"
