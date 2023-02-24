TARGET_ELF_RELEASE = $(BLD_RELEASE_DIR)/nucleof446re_rel.elf
TARGET_MAP_RELEASE = $(BLD_RELEASE_DIR)/nucleof446re_rel.map
TARGET_ELF_DEBUG = $(BLD_DEBUG_DIR)/nucleof446re_dbg.elf
TARGET_MAP_DEBUG = $(BLD_DEBUG_DIR)/nucleof446re_dbg.map
TARGET_LIB = $(LIB_DIR)/libstm32f446xx.a

VPATH = $(shell find ./src/ -type d)
LIB_DIR = ./lib
BLD_DIR = ./build
BLD_RELEASE_DIR = $(BLD_DIR)/release
BLD_DEBUG_DIR = $(BLD_DIR)/debug
OBJ_RELEASE_DIR = $(BLD_RELEASE_DIR)/obj
OBJ_DEBUG_DIR = $(BLD_DEBUG_DIR)/obj
OBJ_LIB_DIR = $(LIB_DIR)/obj
LNK_DIR = ./lnk

INCLUDE = $(VPATH:%=-I%) 
SOURCES = $(shell find ./src/ -type f -name "*.c")
OBJECTS = $(notdir $(patsubst %.c, %.o, $(SOURCES)))
OBJS_DRV = $(notdir $(patsubst %.c, %.o, $(filter ./src/drv/%, $(SOURCES))))
OBJS_RELEASE = $(addprefix $(OBJ_RELEASE_DIR)/, $(OBJECTS))
OBJS_DEBUG = $(addprefix $(OBJ_DEBUG_DIR)/, $(filter-out %syscalls.o, $(OBJECTS)))
OBJS_LIB = $(addprefix $(OBJ_LIB_DIR)/, $(OBJS_DRV))

MODE = all

CC = arm-none-eabi-gcc
AR = arm-none-eabi-ar
CR = arm-none-eabi-ranlib
MACH = cortex-m4
CFLAGS = -c -MD -mcpu=$(MACH) -mthumb -mfloat-abi=soft -std=gnu11 -Wall $(INCLUDE) -O0
LDFLAGS_RELEASE = -mcpu=$(MACH) -mthumb -mfloat-abi=soft --specs=nano.specs -T $(LNK_DIR)/lk_f446re.ld \
				  -Wl,-Map=$(TARGET_MAP_RELEASE) -Wl,--print-memory-usage
LDFLAGS_DEBUG = -mcpu=$(MACH) -mthumb -mfloat-abi=soft --specs=rdimon.specs -T $(LNK_DIR)/lk_f446re.ld \
				  -Wl,-Map=$(TARGET_MAP_DEBUG) -Wl,--print-memory-usage


$(TARGET_ELF_RELEASE) : CFLAGS += -DRELEASE
$(TARGET_ELF_RELEASE) : $(OBJS_RELEASE)
	@mkdir -p $(BLD_DIR)
	$(CC) $(LDFLAGS_RELEASE) $(OBJS_RELEASE) -o $(TARGET_ELF_RELEASE)

$(TARGET_ELF_DEBUG) : CFLAGS += -DDEBUG -g
$(TARGET_ELF_DEBUG) : $(OBJS_DEBUG)
	@mkdir -p $(BLD_DIR)
	$(CC) $(LDFLAGS_DEBUG) $(OBJS_DEBUG) -o $(TARGET_ELF_DEBUG)

$(TARGET_LIB) : CFLAGS += -DRELEASE
$(TARGET_LIB) : $(OBJS_LIB)
	@mkdir -p $(LIB_DIR)
	$(AR) -rc $@ $+
	$(CR) $@

$(OBJ_RELEASE_DIR)/%.o : %.c
	@mkdir -p $(OBJ_RELEASE_DIR)
	$(CC) $(CFLAGS) $< -o $@

$(OBJ_DEBUG_DIR)/%.o : %.c
	@mkdir -p $(OBJ_DEBUG_DIR)
	$(CC) $(CFLAGS) $< -o $@

$(OBJ_LIB_DIR)/%.o : %.c
	@mkdir -p $(OBJ_LIB_DIR)
	$(CC) $(CFLAGS) $< -o $@

-include $(OBJ_RELEASE_DIR)/*.d
-include $(OBJ_DEBUG_DIR)/*.d

.PHONY all:
all: $(TARGET_ELF_DEBUG) \
	 $(TARGET_ELF_RELEASE) \
	 $(TARGET_LIB)

.PHONY : release
release: $(TARGET_ELF_RELEASE)

.PHONY : debug
debug: $(TARGET_ELF_DEBUG)

.PHONY : lib
lib: $(TARGET_LIB)

clean: $(addprefix clean-, $(MODE))

.PHONY : clean-all
clean-all:
	@rm -rf $(BLD_DIR) $(LIB_DIR)

.PHONY : clean-debug
clean-debug:
	@rm -rf $(BLD_DEBUG_DIR)

.PHONY : clean-release
clean-release:
	@rm -rf $(BLD_RELEASE_DIR)

.PHONY : clean-lib
clean-lib:
	@rm -rf $(LIB_DIR)

.PHONY : load
load:
	openocd -f board/st_nucleo_f4.cfg