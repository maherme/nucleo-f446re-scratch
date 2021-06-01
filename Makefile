CC=arm-none-eabi-gcc
MACH=cortex-m4
CFLAGS= -c -mcpu=$(MACH) -mthumb -mfloat-abi=soft -std=gnu11 -Wall -O0
LDFLAGS= -mcpu=$(MACH) -mthumb -mfloat-abi=soft -specs=nano.specs -T lk_f446re.ld -Wl,-Map=final.map

all: main.o startup.o syscalls.o final.elf

main.o: main.c
	$(CC) $(CFLAGS) $^ -o $@

startup.o: startup.c
	$(CC) $(CFLAGS) $^ -o $@

syscalls.o: syscalls.c
	$(CC) $(CFLAGS) $^ -o $@

final.elf: main.o startup.o syscalls.o
	$(CC) $(LDFLAGS) $^ -o $@

clean:
	rm -rf *.o *.elf *.map

load:
	openocd -f board/st_nucleo_f4.cfg
