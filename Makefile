# Ergenon-Systems Build System
# Author: Batuhan ALGÜL


TARGET = stm32h743
ARCH = cortex-m4
FPU = fpv4-sp-d16


CC = arm-none-eabi-gcc
AS = arm-none-eabi-gcc
LD = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size


SRC_DIR = src
BUILD_DIR = build
BIN_DIR = bin


CFLAGS = -mcpu=$(ARCH) -mthumb -mfpu=$(FPU) -mfloat-abi=hard
CFLAGS += -Wall -Wextra -Werror -pedantic
CFLAGS += -O2 -g
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -I$(SRC_DIR)
CFLAGS += -DMCU_$(TARGET)


ASFLAGS = -mcpu=$(ARCH) -mthumb -mfpu=$(FPU)
ASFLAGS += -g


LDFLAGS = -mcpu=$(ARCH) -mthumb -mfpu=$(FPU) -mfloat-abi=hard
LDFLAGS += -T$(SRC_DIR)/linker.ld
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -Wl,-Map=$(BUILD_DIR)/ergenon.map


C_SOURCES = $(SRC_DIR)/kernel.c
ASM_SOURCES = $(SRC_DIR)/boot.s


C_OBJECTS = $(C_SOURCES:$(SRC_DIR)/%.c=$(BUILD_DIR)/%.o)
ASM_OBJECTS = $(ASM_SOURCES:$(SRC_DIR)/%.s=$(BUILD_DIR)/%.o)
OBJECTS = $(C_OBJECTS) $(ASM_OBJECTS)


TARGET_ELF = $(BIN_DIR)/ergenon.elf
TARGET_BIN = $(BIN_DIR)/ergenon.bin
TARGET_HEX = $(BIN_DIR)/ergenon.hex


all: directories $(TARGET_BIN) $(TARGET_HEX) size


directories:
	@mkdir -p $(BUILD_DIR)
	@mkdir -p $(BIN_DIR)


$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) $(CFLAGS) -c $< -o $@


$(BUILD_DIR)/%.o: $(SRC_DIR)/%.s
	$(AS) $(ASFLAGS) -c $< -o $@


$(TARGET_ELF): $(OBJECTS)
	$(LD) $(LDFLAGS) $(OBJECTS) -o $@


$(TARGET_BIN): $(TARGET_ELF)
	$(OBJCOPY) -O binary $< $@


$(TARGET_HEX): $(TARGET_ELF)
	$(OBJCOPY) -O ihex $< $@


size: $(TARGET_ELF)
	@echo "=== Memory Usage ==="
	$(SIZE) -A $<


clean:
	rm -rf $(BUILD_DIR)
	rm -rf $(BIN_DIR)


flash: $(TARGET_ELF)
	openocd -f interface/stlink.cfg -f target/stm32h7x.cfg \
		-c "program $(TARGET_ELF) verify reset exit"


debug: $(TARGET_ELF)
	openocd -f interface/stlink.cfg -f target/stm32h7x.cfg \
		-c "gdb_port 3333" -c "tcl_port 3334" -c "telnet_port 3335"


rust:
	cd rust && cargo build --release --target thumbv7em-none-eabihf


full: all rust
	@echo "=== Full Build Complete ==="

.PHONY: all clean flash debug size directories rust full
