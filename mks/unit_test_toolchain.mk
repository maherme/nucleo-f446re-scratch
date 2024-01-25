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
GCOVRFLAGS = --html-details \
             --exclude $(DIR_MCU) \
             -o $(DIR_TEST_COVERAGE)/coverage_report.html \
             --json-summary \
             $(DIR_TEST_COVERAGE)/coverage_report.json
