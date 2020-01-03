CC=g++


CFLAGS= -Iheaders -std=gnu++11 -Wall
LDFLAGS= `pkg-config --cflags --libs allegro-5 allegro_primitives-5` -lallegro -lrt -lm -lpthread
all: Engine

#Engine: pch.o GameLoop.o Engine.o
#	$(CC) pch.o GameLoop.o Engine.o -o Engine  $(LDFLAGS) 

Engine: pch.cpp GameLoop.cpp Engine.cpp
	$(CC) $(CFLAGS) pch.cpp GameLoop.cpp Engine.cpp -o Engine  $(LDFLAGS) 

#pch.o: pch.cpp
#	$(CC) $(CFLAGS) pch.cpp

#GameLoop.o: GameLoop.cpp
#	$(CC) $(CFLAGS) GameLoop.cpp

#Engine.o: Engine.cpp
#	$(CC) $(CFLAGS) Engine.cpp

clean:
	rm -rf *o Engine

