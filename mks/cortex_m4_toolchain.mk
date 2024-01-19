export CC := arm-none-eabi-gcc
export AR := arm-none-eabi-ar
export CR := arm-none-eabi-ranlib
export MCPU := cortex-m4
export CFLAGS := -c \
                 -MD \
                 -mcpu=$(MCPU) \
                 -mthumb \
                 -mfloat-abi=soft \
                 -std=gnu11 \
                 -Wall \
                 -Wextra \
                 -Werror \
                 $(INCLUDE) \
                 -O0
export LDFLAGS = -mcpu=$(MCPU) \
                 -mthumb \
                 -mfloat-abi=soft \
                 -T $(DIR_LINKER)/lk_f446re.ld \
                 -Wl,-Map=$(MAP_FILE) \
                 -Wl,--print-memory-usage
