CC=gcc
CFLAGS=-lwiringPi
app=spiApp
lib=spiADE9078

$(app): $(app).o $(lib).o 
	$(CC) $(app).o $(lib).o -o ./build/$(app) $(CFLAGS) 
	./build/$(app) /dev/spidev0.0

$(app).o: $(app).c
	$(CC) -c $(app).c

$(lib).o: $(lib).c $(lib).h
	$(CC) -c $(lib).c

