TARGET1 = $(BLD_DIR)/nucleof446re_rel.elf
TARGET2 = $(BLD_DIR)/nucleof446re_dbg.elf
TARGET_LIB = $(LIB_DIR)/libstm32f446xx.a
SRC_DIR = ./src
DRV_DIR = ./src/drv/*/
TST_DIR = ./src/tst
TST_SUBDIR = ./src/tst/*/
UTL_DIR = ./src/utl
INCLUDE = -I./inc \
		  -I./src/utl \
		  -I./src/tst \
		  -I./src/tst/spi \
		  -I./src/tst/i2c \
		  -I./src/tst/usart \
		  -I./src/tst/rcc \
		  -I./src/tst/timer \
		  -I./src/tst/dma \
		  -I./src/tst/rtc \
		  -I./src/tst/can \
		  -I./src/drv \
		  -I./src/drv/gpio \
		  -I./src/drv/spi \
		  -I./src/drv/i2c \
		  -I./src/drv/usart \
		  -I./src/drv/flash \
		  -I./src/drv/rcc \
		  -I./src/drv/timer \
		  -I./src/drv/dma \
		  -I./src/drv/rtc \
		  -I./src/drv/can
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
		$(OBJ_DIR)/test_usart.o \
		$(OBJ_DIR)/test_rcc.o \
		$(OBJ_DIR)/test_timer.o \
		$(OBJ_DIR)/test_dma.o \
		$(OBJ_DIR)/test_rtc.o \
		$(OBJ_DIR)/test_can.o \
		$(OBJ_DIR)/gpio_driver.o \
		$(OBJ_DIR)/spi_driver.o \
		$(OBJ_DIR)/rcc_driver.o \
		$(OBJ_DIR)/i2c_driver.o \
		$(OBJ_DIR)/usart_driver.o \
		$(OBJ_DIR)/flash_driver.o \
		$(OBJ_DIR)/timer_driver.o \
		$(OBJ_DIR)/dma_driver.o \
		$(OBJ_DIR)/rtc_driver.o \
		$(OBJ_DIR)/can_driver.o
OBJS2 = $(OBJ_DIR)/main.o \
		$(OBJ_DIR)/startup.o \
		$(OBJ_DIR)/utils.o \
		$(OBJ_DIR)/test.o \
		$(OBJ_DIR)/test_spi.o \
		$(OBJ_DIR)/test_i2c.o \
		$(OBJ_DIR)/test_usart.o \
		$(OBJ_DIR)/test_rcc.o \
		$(OBJ_DIR)/test_timer.o \
		$(OBJ_DIR)/test_dma.o \
		$(OBJ_DIR)/test_rtc.o \
		$(OBJ_DIR)/test_can.o \
		$(OBJ_DIR)/gpio_driver.o \
		$(OBJ_DIR)/spi_driver.o \
		$(OBJ_DIR)/rcc_driver.o \
		$(OBJ_DIR)/i2c_driver.o \
		$(OBJ_DIR)/usart_driver.o \
		$(OBJ_DIR)/flash_driver.o \
		$(OBJ_DIR)/timer_driver.o \
		$(OBJ_DIR)/dma_driver.o \
		$(OBJ_DIR)/rtc_driver.o \
		$(OBJ_DIR)/can_driver.o
OBJS_LIB = $(OBJ_DIR)/gpio_driver.o \
		   $(OBJ_DIR)/spi_driver.o \
		   $(OBJ_DIR)/rcc_driver.o \
		   $(OBJ_DIR)/i2c_driver.o \
		   $(OBJ_DIR)/usart_driver.o \
		   $(OBJ_DIR)/flash_driver.o \
		   $(OBJ_DIR)/timer_driver.o \
		   $(OBJ_DIR)/dma_driver.o \
		   $(OBJ_DIR)/rtc_driver.o \
		   $(OBJ_DIR)/can_driver.o
CC = arm-none-eabi-gcc
AR = arm-none-eabi-ar
CR = arm-none-eabi-ranlib
MACH = cortex-m4
CFLAGS = -c -MD -mcpu=$(MACH) -mthumb -mfloat-abi=soft -std=gnu11 -Wall $(INCLUDE) -O0
LDFLAGS = -mcpu=$(MACH) -mthumb -mfloat-abi=soft --specs=nano.specs -T $(LNK_DIR)/lk_f446re.ld \
		  -Wl,-Map=$(BLD_DIR)/nucleof446re.map -Wl,--print-memory-usage
LDFLAGS_SH = -mcpu=$(MACH) -mthumb -mfloat-abi=soft --specs=rdimon.specs -T $(LNK_DIR)/lk_f446re.ld \
			 -Wl,-Map=$(BLD_DIR)/nucleof446re_sh.map -Wl,--print-memory-usage

$(TARGET_LIB) : $(OBJS_LIB)
	@mkdir -p $(LIB_DIR)
	$(AR) -rc $@ $+
	$(CR) $@

$(TARGET1) : CFLAGS += -DRELEASE
$(TARGET1) : $(OBJS1)
	@mkdir -p $(BLD_DIR)
	$(CC) $(LDFLAGS) $(OBJS1) -o $(TARGET1)

$(TARGET2) : CFLAGS += -DDEBUG -g
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

$(OBJ_DIR)/%.o : $(TST_SUBDIR)/%.c
	@mkdir -p $(OBJ_DIR)
	$(CC) $(CFLAGS) $< -o $@

$(OBJ_DIR)/%.o : $(UTL_DIR)/%.c
	@mkdir -p $(OBJ_DIR)
	$(CC) $(CFLAGS) $< -o $@

-include $(OBJ_DIR)/*.d

.PHONY : release
release: $(TARGET1)

.PHONY : debug
debug: $(TARGET2)

.PHONY : clean
clean:
	rm -r $(OBJ_DIR) $(BLD_DIR) $(LIB_DIR)

.PHONY : lib
lib: $(TARGET_LIB)

.PHONY : load
load:
	openocd -f board/st_nucleo_f4.cfg
