TARGET1 = $(BLD_DIR)/nucleof446re.elf
TARGET2 = $(BLD_DIR)/nucleof446re_sh.elf
TARGET_LIB = $(LIB_DIR)/libstm32f446xx.a
SRC_DIR = ./src
DRV_DIR = ./src/drv
TST_DIR = ./src/tst
INCLUDE = -I./inc -I./inc/drv -I./inc/tst
LNK_DIR = ./lnk
OBJ_DIR = ./obj
LIB_DIR = ./lib
BLD_DIR = ./build
OBJS1 = $(OBJ_DIR)/main.o \
		$(OBJ_DIR)/startup.o \
		$(OBJ_DIR)/syscalls.o \
		$(OBJ_DIR)/utils.o \
		$(OBJ_DIR)/test.o \
		$(OBJ_DIR)/test_spi.o \
		$(OBJ_DIR)/test_i2c.o \
		$(OBJ_DIR)/gpio_driver.o \
		$(OBJ_DIR)/spi_driver.o \
		$(OBJ_DIR)/rcc_driver.o \
		$(OBJ_DIR)/i2c_driver.o \
		$(OBJ_DIR)/usart_driver.o \
		$(OBJ_DIR)/flash_driver.o
OBJS2 = $(OBJ_DIR)/main.o \
		$(OBJ_DIR)/startup.o \
		$(OBJ_DIR)/utils.o \
		$(OBJ_DIR)/test.o \
		$(OBJ_DIR)/test_spi.o \
		$(OBJ_DIR)/test_i2c.o \
		$(OBJ_DIR)/gpio_driver.o \
		$(OBJ_DIR)/spi_driver.o \
		$(OBJ_DIR)/rcc_driver.o \
		$(OBJ_DIR)/i2c_driver.o \
		$(OBJ_DIR)/usart_driver.o \
		$(OBJ_DIR)/flash_driver.o
OBJS_LIB = $(OBJ_DIR)/gpio_driver.o \
		   $(OBJ_DIR)/spi_driver.o \
		   $(OBJ_DIR)/rcc_driver.o \
		   $(OBJ_DIR)/i2c_driver.o \
		   $(OBJ_DIR)/usart_driver.o \
		   $(OBJ_DIR)/flash_driver.o
CC = arm-none-eabi-gcc
AR = arm-none-eabi-ar
CR = arm-none-eabi-ranlib
MACH = cortex-m4
CFLAGS = -c -MD -mcpu=$(MACH) -mthumb -mfloat-abi=soft -std=gnu11 -Wall $(INCLUDE) -O0
LDFLAGS = -mcpu=$(MACH) -mthumb -mfloat-abi=soft --specs=nano.specs -T $(LNK_DIR)/lk_f446re.ld \
		  -Wl,-Map=$(BLD_DIR)/nucleof446re.map
LDFLAGS_SH = -mcpu=$(MACH) -mthumb -mfloat-abi=soft --specs=rdimon.specs -T $(LNK_DIR)/lk_f446re.ld \
			 -Wl,-Map=$(BLD_DIR)/nucleof446re_sh.map

$(TARGET_LIB) : $(OBJS_LIB)
	@mkdir -p $(LIB_DIR)
	$(AR) -rc $@ $+
	$(CR) $@

$(TARGET1) : $(OBJS1)
	@mkdir -p $(BLD_DIR)
	$(CC) $(LDFLAGS) $(OBJS1) -o $(TARGET1)

$(TARGET2) : $(OBJS2)
	@mkdir -p $(BLD_DIR)
	$(CC) $(LDFLAGS_SH) $(OBJS2) -o $(TARGET2)

$(OBJ_DIR)/%.o : $(SRC_DIR)/%.c
	@mkdir -p $(OBJ_DIR)
	$(CC) $(CFLAGS) $< -o $@

$(OBJ_DIR)/%.o : $(DRV_DIR)/%.c
	@mkdir -p $(OBJ_DIR)
	$(CC) $(CFLAGS) $< -o $@

$(OBJ_DIR)/%.o : $(TST_DIR)/%.c
	@mkdir -p $(OBJ_DIR)
	$(CC) $(CFLAGS) $< -o $@

-include $(OBJ_DIR)/*.d

.PHONY : all
all: $(TARGET1)

.PHONY : semi
semi: $(TARGET2)

.PHONY : clean
clean:
	rm -r $(OBJ_DIR) $(BLD_DIR) $(LIB_DIR)

.PHONY : lib
lib: $(TARGET_LIB)

.PHONY : load
load:
	openocd -f board/st_nucleo_f4.cfg
