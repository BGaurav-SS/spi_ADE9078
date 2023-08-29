CC=gcc
CFLAGS=-lwiringPi
example=exampleApplication
lib=spiADE9078

example: $(example).o $(lib).o
	$(CC) $(example).o $(lib).o -o ./build/$(example) $(CFLAGS)
	./build/$(example) /dev/spidev0.0

$(example).o: $(example).c
	$(CC) -c $(example).c

$(lib).o: $(lib).c $(lib).h
	$(CC) -c $(lib).c
