DIR_ROOT_BUILD := $(CURDIR)/build
DIR_SOURCES := $(CURDIR)/src
DIR_DRIVERS := $(DIR_SOURCES)/drv
DIR_MCU := $(DIR_SOURCES)/mcu
DIR_UTILS := $(DIR_SOURCES)/utl
DIR_CPPUTEST := $(CURDIR)/cpputest
DIR_LINKER := $(CURDIR)/lnk

COMPONENTS := can \
              dma \
              flash \
              gpio \
              i2c \
              rcc \
              rtc \
              spi \
              timer \
              usart \
              pwr

SOURCES_MAIN := $(wildcard $(DIR_SOURCES)/*.c)
SOURCES_TEST_MAIN := $(wildcard $(DIR_SOURCES)/*.cpp)
SOURCES_MCU := $(wildcard $(DIR_MCU)/*.c)
SOURCES_UTILS := $(wildcard $(DIR_UTILS)/*.c)

DIR_SOURCES_COMPONENTS := $(addsuffix /src,$(addprefix $(DIR_DRIVERS)/,$(COMPONENTS)))
DIR_TEST_COMPONENTS := $(addsuffix /tst,$(addprefix $(DIR_DRIVERS)/,$(COMPONENTS)))
DIR_HIL_COMPONENTS := $(addsuffix /hil,$(addprefix $(DIR_DRIVERS)/,$(COMPONENTS)))

VPATH := $(DIR_SOURCES) \
         $(DIR_MCU) \
         $(DIR_UTILS) \
         $(DIR_SOURCES_COMPONENTS) \
         $(DIR_TEST_COMPONENTS) \
         $(DIR_HIL_COMPONENTS)
INCLUDE := $(VPATH:%=-I%) -I$(DIR_CPPUTEST)/include

OBJ_MAIN := $(notdir $(patsubst %.c, %.o, $(SOURCES_MAIN)))
OBJ_TEST_MAIN := $(notdir $(patsubst %.cpp, %.o, $(SOURCES_TEST_MAIN)))
OBJ_MCU := $(notdir $(patsubst %.c, %.o, $(SOURCES_MCU)))
OBJ_UTILS := $(notdir $(patsubst %.c, %.o, $(SOURCES_UTILS)))

export LD_LIBRARIES := -L$(DIR_CPPUTEST)/cpputest_build/lib -lCppUTest \
                       -L$(DIR_CPPUTEST)/cpputest_build/lib -lCppUTestExt
