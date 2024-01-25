PROJECT_NAME := stm32f446xx

export MKDIR := mkdir -p
export RM := rm -rf
export QUIET := @
QUIET_MAKE := --no-print-directory
# These variable will be filled during the make flow
DIR_NEEDS :=

include mks/configuration.mk

ifeq ($(MODE), debug)
    include mks/cortex_m4_toolchain.mk
    DIR_TARGET_BUILD := $(DIR_ROOT_BUILD)/debug
    OBJECTS := $(filter-out syscalls.o,$(OBJ_MAIN)) $(OBJ_MCU) $(OBJ_UTILS)
    DEPENDENCIES = $(COMPONENTS) $(OBJECTS)
    CFLAGS += -g
    LDFLAGS += --specs=rdimon.specs
    CMD_BUILD = $(CC) $(LDFLAGS) $(DIR_OBJS)/*.o -o $@
else ifeq ($(MODE), test)
    include mks/unit_test_toolchain.mk
    DIR_TARGET_BUILD := $(DIR_ROOT_BUILD)/test
    DIR_TEST_COVERAGE := $(DIR_TARGET_BUILD)/coverage
    DIR_NEEDS += $(DIR_TEST_COVERAGE)
    OBJECTS := $(OBJ_TEST_MAIN) $(OBJ_MCU)
    DEPENDENCIES = $(COMPONENTS) $(OBJECTS)
    CFLAGS += --coverage
    CMD_BUILD = $(CXX) $(LDFLAGS) $(DIR_OBJS)/*.o -o $@ $(LD_LIBRARIES)
else ifeq ($(MODE), set-test-env)
    include mks/host_toolchain.mk
else
    include mks/cortex_m4_toolchain.mk
    DIR_TARGET_BUILD := $(DIR_ROOT_BUILD)/lib
    TARGET := $(DIR_TARGET_BUILD)/lib$(PROJECT_NAME).a
    DEPENDENCIES = $(COMPONENTS)
    LDFLAGS += --specs=nano.specs
    CMD_BUILD = $(AR) -rc $@ $(DIR_OBJS)/*.o && $(CR) $@
endif

DIR_OBJS := $(DIR_TARGET_BUILD)/obj
DIR_NEEDS += $(DIR_OBJS)
OBJECTS := $(addprefix $(DIR_OBJS)/,$(OBJECTS))
TARGET ?= $(DIR_TARGET_BUILD)/$(PROJECT_NAME).elf
MAP_FILE := $(DIR_TARGET_BUILD)/$(PROJECT_NAME).map

$(shell $(MKDIR) $(DIR_NEEDS))

$(TARGET): $(DEPENDENCIES)
	$(QUIET) $(CMD_BUILD)

$(COMPONENTS):
	$(QUIET) $(MAKE) $(QUIET_MAKE) --directory=$(DIR_DRIVERS)/$@ OUTPUT_DIR=$(DIR_OBJS)

$(DIR_OBJS)/%.o: %.c
	$(QUIET) $(CC) $(CFLAGS) $< -o $@

$(DIR_OBJS)/%.o: %.cpp
	$(QUIET) $(CC) $(CXXFLAGS) $< -o $@

-include $(DIR_OBJS)/*.d

.PHONY : all
all: library debug_bin unit_test

.PHONY: library
library:
	@echo Generating Library
	$(QUIET) $(MAKE) $(QUIET_MAKE)

.PHONY: debug_bin
debug_bin:
	@echo Generating Debug Binary
	$(QUIET) $(MAKE) $(QUIET_MAKE) MODE=debug

.PHONY: unit_test
unit_test:
	@echo Generating Unitary Tests
	$(QUIET) $(MAKE) $(QUIET_MAKE) MODE=test

.PHONY: clean-all
clean-all:
	$(QUIET) $(RM) $(DIR_ROOT_BUILD)

.PHONY: clean
clean:
	$(QUIET) $(RM) $(DIR_TARGET_BUILD)

.PHONY: load
load:
	openocd -f board/st_nucleo_f4.cfg

.PHONY: set-test-env
set-test-env:
	$(QUIET) mkdir -p $(DIR_CPPUTEST)/cpputest_build
	$(QUIET) cd $(DIR_CPPUTEST)/cpputest_build; \
	$(QUIET) autoreconf .. -i; \
	$(QUIET) ../configure; \
	$(QUIET) make

.PHONY: run-test
run-test: unit_test
	$(QUIET) $(TARGET)
	$(QUIET) $(MAKE) $(QUIET_MAKE) gen-coverage

.PHONY: gen-coverage
gen-coverage:
	$(QUIET) $(GCOVR) $(GCOVRFLAGS) -o $(DIR_TEST_COVERAGE)/coverage_report.html
