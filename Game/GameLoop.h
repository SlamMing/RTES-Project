#pragma once
#ifndef GAMELOOP_H
#define GAMELOOP_H
class GameLoop {
private:
	static bool running;
public:
	static void setRunning(bool b);
	static bool isRunning();
};


#endif