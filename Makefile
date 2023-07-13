PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

SEL = 0

# ifeq ($(SEL), 0)
# SRC_DIR = src
# else ($(SEL), 1)
# SRC_DIR = src/lab3
# endif

# Directories
SRC_DIR = src
OBJ_DIR = build
INC_DIR = \
	-Iinclude \
	-I/usr/arm-none-eabi/include
STARTUP_DIR = startup

# Files 
SRC := $(wildcard $(SRC_DIR)/*.c)
SRC += $(wildcard $(STARTUP_DIR)/*.c)
OBJ := $(patsubst $(SRC_DIR)/%.c, $(OBJ_DIR)/%.o, $(SRC))
OBJ := $(patsubst $(STARTUP_DIR)/%.c, $(OBJ_DIR)/%.o, $(OBJ))
LD := $(wildcard $(STARTUP_DIR)/*.ld)

COREFLAGS=-mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=softfp
CFLAGS = $(COREFLAGS)
CFLAGS += -Ofast -funroll-loops -ffunction-sections -fdata-sections -Wall -save-temps -Wno-unused-function
CFLAGS += -I$(INC_DIR)
LFLAGS = $(COREFLAGS) -T$(LD) -Wl,-Map=$(OBJ_DIR)/main.map,--gc-sections

# FLAGS
# MARCH = cortex-m4
# LIBS = -lc -lm -lnosys
# CFLAGS =  -ffreestanding -nostartfiles -mcpu=$(MARCH) -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -O3 $(INC_DIR) -Wall -fdata-sections -ffunction-sections 
# LFLAGS = -nostdlib -T $(LD) $(LIBS) -Wl,-Map=$(OBJ_DIR)/main.map

#PATHS
OPENOCD_INTERFACE = /usr/share/openocd/scripts/interface/stlink.cfg
OPENOCD_TARGET = /usr/share/openocd/scripts/target/stm32f3x.cfg

# Targets
TARGET = $(OBJ_DIR)/main.elf

all: $(OBJ) $(TARGET)

$(OBJ_DIR)/%.o : $(SRC_DIR)/%.c | mkobj
	$(CC) -c $(CFLAGS) -o $@ $^

$(OBJ_DIR)/%.o : $(STARTUP_DIR)/%.c | mkobj
	$(CC) -c $(CFLAGS) -o $@ $^

$(TARGET) : $(OBJ) | mkobj
	$(CC) $(CFLAGS) $(LFLAGS) -o $@ $^

mkobj:
	mkdir -p $(OBJ_DIR)

flash:
	openocd -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) -c \
	"program $(TARGET) verify reset exit"

clean:
	rm -rf $(OBJ_DIR)

force: clean all flash

.PHONY = mkobj clean force flash