OUTPUT_DIR ?= .
COMPONENT_NAME := $(notdir $(CURDIR))

ifeq ($(MODE), debug)
    LOCAL_SOURCES := $(wildcard $(CURDIR)/src/*.c) \
                     $(wildcard $(CURDIR)/hil/*.c)
else ifeq ($(MODE), test)
    LOCAL_SOURCES := $(wildcard $(CURDIR)/src/*.c) \
                     $(wildcard $(CURDIR)/../../*.cpp) \
                     $(wildcard $(CURDIR)/tst/*.cpp)
else
    LOCAL_SOURCES := $(wildcard $(CURDIR)/src/*.c)
endif

ifeq ($(UNIT_TEST), each)
    LOCAL_CMD = $(CXX) $^ -o $(OUTPUT_DIR)/../$@_$(COMPONENT_NAME) $(LD_LIBRARIES)
endif

LOCAL_OBJECTS := $(addprefix $(OUTPUT_DIR)/, $(notdir $(patsubst %.c,%.o,$(filter %.c,$(LOCAL_SOURCES)))))
LOCAL_OBJECTS += $(addprefix $(OUTPUT_DIR)/,$(notdir $(patsubst %.cpp,%.o,$(filter %.cpp,$(LOCAL_SOURCES)))))
VPATH := $(dir $(LOCAL_SOURCES))

$(COMPONENT_NAME) : $(LOCAL_OBJECTS)
	$(QUIET) $(LOCAL_CMD)

$(OUTPUT_DIR)/%.o : %.c
	$(QUIET) $(CC) $(CFLAGS) $< -o $@

$(OUTPUT_DIR)/%.o : %.cpp
	$(QUIET) $(CC) $(CXXFLAGS) $< -o $@
