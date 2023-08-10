CC=gcc
CFLAGS=-lwiringPi
file=spiKernelLevel

$(file): $(file).c
	$(CC) $(file).c -o ./build/$(file) $(CFLAGS)
	./build/$(file) /dev/spidev0.0

build: $(file).c
	$(CC) $(file).c -o ./build/$(file) $(CFLAGS)

speed: spiSpeed.c
	$(CC) spiSpeed.c -o ./build/spiSpeed $(CFLAGS)
	./build/spiSpeed -D /dev/spidev0.0

wiringPi: spiWiringPi.c
	$(CC) spiWiringPi.c -o ./build/spiWiringPi $(CFLAGS)
	./build/spiWiringPi 