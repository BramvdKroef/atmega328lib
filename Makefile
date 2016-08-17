#########################################
# Configuration
#########################################

# Directory containing source files
SRC_DIR = src
# Directory to place compiled .o files.
BUILD_DIR = build

BINARY = binary

# Serial port for uploading.
SERIAL_PORT = /dev/ttyUSB0

MCU = atmega328p
F_CPU = 16000000L
AVRDUDE_PROTOCOL = arduino

#########################################
# Build tools
#########################################

CC      = avr-gcc
CXX     = avr-g++
OBJCOPY = avr-objcopy
STRIP   = avr-strip
AVRDUDE = avrdude
SIZE	= avr-size

#########################################
# Project sources
#########################################

C_SRC   = $(shell ls $(SRC_DIR)/*.c   2> /dev/null)
OBJ := $(addprefix $(BUILD_DIR)/,$(notdir $(C_SRC:.c=.o)))

#########################################
# Compiler variables
#########################################

INC_DIRS := -I $(SRC_DIR)
COMPILER_FLAGS = -Wall -Os -funsigned-char -funsigned-bitfields\
-fpack-struct -fno-exceptions
CFLAGS_HW := -mmcu=$(MCU) -DF_CPU=$(F_CPU)
CFLAGS   := $(COMPILER_FLAGS) $(CFLAGS_HW) $(INC_DIRS) 
export CFLAGS

# Avrdude hardware flags
AVRDUDE_HW := -p$(MCU) -c$(AVRDUDE_PROTOCOL) -P$(SERIAL_PORT)


.PHONY: all
all: $(BINARY).hex

.PHONY: clean
clean: 
	@$(RM) -v $(BINARY).elf
	@$(RM) -v $(BINARY).hex
	@$(RM) -v $(BUILD_DIR)/*.o
	@rmdir -v $(BUILD_DIR)


$(BUILD_DIR):
	mkdir $(BUILD_DIR)

$(BINARY).hex: $(BINARY).elf
	$(STRIP) -s $(BINARY).elf
	$(OBJCOPY) -O ihex -R .eeprom $(BINARY).elf $(BINARY).hex

$(BINARY).elf: $(BUILD_DIR) $(OBJ)
	$(CC) $(LDFLAGS) $(OBJ) -o $(BINARY).elf

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	$(COMPILE.c) $(OUTPUT_OPTION) $<

.PHONY: upload
upload: $(BINARY).hex
	stty -F $(SERIAL_PORT) hupcl
	$(AVRDUDE) $(AVRDUDE_HW) -D -Uflash:w:$(BINARY).hex:i

.PHONY: monitor
monitor:
	stty -F $(SERIAL_PORT) cs8 9600 ignbrk -brkint -icrnl -imaxbel -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke noflsh -ixon -crtscts 
	cat $(SERIAL_PORT)
