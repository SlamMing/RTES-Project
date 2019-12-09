CC=g++


CFLAGS=-c -Iheaders -std=gnu++11
LDFLAGS= `pkg-config --cflags --libs allegro-5 allegro_primitives-5` -lallegro -lrt -lm -lpthread
all: Game

Game: pch.o Game.o GameLoop.o Engine.o
	$(CC) pch.o GameLoop.o Game.o Engine.o -o Game  $(LDFLAGS) 

pch.o: pch.cpp
	$(CC) $(CFLAGS) pch.cpp

Game.o: Game.cpp
	$(CC) $(CFLAGS) Game.cpp

GameLoop.o: GameLoop.cpp
	$(CC) $(CFLAGS) GameLoop.cpp

Engine.o: Engine.cpp
	$(CC) $(CFLAGS) Engine.cpp

clean:
	rm -rf *o Game

