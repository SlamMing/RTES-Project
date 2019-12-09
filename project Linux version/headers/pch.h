#ifndef PCH_H
#define PCH_H
#define _TIMESPEC_DEFINED
#define _CRT_SECURE_NO_WARNINGS
#include <allegro5/allegro.h>
#include <allegro5/allegro_primitives.h>
#include <pthread.h>
#include <semaphore.h>
#include <string>
#include <iostream>
#include <fstream>
#include "GameLoop.h"

//#include <windows.h> 
constexpr auto MAXSPEED = 5;
constexpr auto HEIGHT = 960;
constexpr auto WIDTH = 1760;
constexpr auto M_PI = 3.1415926535;
constexpr auto VEHICLE_LENGTH = 20;
constexpr auto FPS = 60;
constexpr auto OPTIMAL_TIME = 1000 / FPS;
constexpr auto CELL_WIDTH = 80;
constexpr auto SENSOR_RANGE = 100;
constexpr auto spawnX = WIDTH/2 + CELL_WIDTH/2;
constexpr auto spawnY = HEIGHT - CELL_WIDTH*3/2;
constexpr auto nWidth = WIDTH / CELL_WIDTH;
constexpr auto nHeight = HEIGHT / CELL_WIDTH;
constexpr auto OPTIMAL = CELL_WIDTH / 2 - VEHICLE_LENGTH / 2;
constexpr auto TEMP = 8;
constexpr auto N = 12;
constexpr auto e = 2.71828182845904523536;

#endif //PCH_H
