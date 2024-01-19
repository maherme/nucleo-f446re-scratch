export CC := gcc
export CFLAGS := -c \
                 -MD \
                 -Wall \
                 -Wextra \
                 -Werror \
                 $(INCLUDE) \
                 -DCOMPILE_OTHER_PLATFORM

export CXX := g++
export CXXFLAGS := -c \
                   -MD \
                   -Wall \
                   -Wextra \
                   -Werror \
                   $(INCLUDE) \
                   -include $(DIR_CPPUTEST)/include/CppUTest/MemoryLeakDetectorNewMacros.h
export LDFLAGS := -lgcov \
				  --coverage

GCOVR := gcovr
GCOVRFLAGS := --html \
              --html-details \
              --exclude $(DIR_MCU)
