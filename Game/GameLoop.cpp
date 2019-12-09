#include "pch.h"

//global access to gameloop
bool GameLoop::running = true;
void GameLoop::setRunning(bool b) {
	GameLoop::running = b;
}

bool GameLoop::isRunning() {
	return running;
}
