CC = arm-none-linux-gnueabi



.PHONY: all clean

all:
	$(CC)-as -g entry.S -o entry.o
	$(CC)-gcc -g -c uart.c -o uart.o -nostdlib
	$(CC)-gcc -g -c gpmi.c -o gpmi.o -nostdlib
	$(CC)-gcc -g -c init.c -o init.o -nostdlib
	$(CC)-gcc -g -c linux.c -o linux.o -nostdlib
	$(CC)-gcc -g -c delay.c -o delay.o -nostdlib
	$(CC)-gcc -g -c cmd.c -o cmd.o -nostdlib
	$(CC)-gcc -g -c str.c -o str.o -nostdlib
#	$(CC)-gcc -g -c dm9000x.c -o dm9000x.o -nostdlib
	$(CC)-gcc -g -c xldr.c -o xldr.o -nostdlib
	$(CC)-gcc -g -c main.c -o main.o -nostdlib
	$(CC)-ld main.o xldr.o init.o linux.o str.o cmd.o delay.o entry.o gpmi.o uart.o -o main.elf -e 0x0 --section-start .reset=0x0 -Ttext 0x1000 -Tdata 0x6000 --reduce-memory-overheads 
	$(CC)-objcopy -O binary main.elf main.bin
	$(CC)-objdump -D -m arm main.elf > main.dis
	ls -lh main.elf main.bin main.dis

clean:
	rm -r *.elf *.bin *~ *.o *.dis
