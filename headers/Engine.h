#ifndef ENGINE_H
#define ENGINE_H
#include <allegro5/allegro.h>
#include <allegro5/allegro_primitives.h>
#include <pthread.h>
#include <semaphore.h>
#include <string>
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <ext/algorithm>

using namespace std;
class Neuron;
typedef vector<Neuron*> Layer;
class AICar;
typedef AICar* AICPtr;
class Sensor;
typedef Sensor* SensPtr;
struct Connection
{
	double weight;
	double dWeight;
};



struct point {
	double x, y;
	point(double x, double y) : x(x), y(y) {}
	point operator-(point p2) {
		point p1(x, y);
		p1.x -= p2.x;
		p1.y -= p2.y;
		return p1;
	}
	point operator+(point p2) {
		point p1(x, y);
		p1.x += p2.x;
		p1.y += p2.y;
		return p1;
	}
	point operator*(point p2) {
		point p1(x, y);
		p1.x *= p2.x;
		p1.y *= p2.y;
		return p1;
	}
	point operator*(double mult) {
		point p1(x, y);
		p1.x *= mult;
		p1.y *= mult;
		return p1;
	}
};
struct line {
	double a, b, c;
	line(point p1, point p2) {
		a = p2.y - p1.y;
		b = p1.x - p2.x;
		c = p1.y * b + p1.x * a;
	}

};

constexpr auto dAngle = M_PI / 60;
constexpr auto dAcc = 0.07;
class Sensor {
	double angle_;
	float lenght;
	float x_, y_;
public:
	Sensor(float x, float y, double angle);
	Sensor() { return;}
	void update(float x, float y, double a);
	double getDistance(double, double);

};
class Car
{
	//vettore movimento
	float sx = 0, sy = 0;
	//vettore accellerazione
	float dx = 0, dy = 0;

	ALLEGRO_COLOR color;
	
	const float MAXACC = 0.1;


public:
	float* pts;
	Car(float, float);
	Car(){ return; }
	~Car();
	void tick();
	void show();
	void move();
	void accelerate(int);
	void turn(int);
	void updatePts();
	float* getPts() { return pts; }
	double getX();
	double getY();
	unsigned getCX();
	unsigned getCY();
	double getAngle();
	void reset();
	//posizione
	float x, y;
	double angle = 0;
	float vel = 0;
};


class EventManager
{
private:
	ALLEGRO_EVENT_QUEUE *queue;
	ALLEGRO_DISPLAY *disp;
	bool pressed = false;
	int mouseX, mouseY;
	bool keys[4] = { false, false, false,false };
	void accellerateUp();
	void accellerateDown();
	void accellerateRight();
	void accellerateLeft();
	void checkMouse(ALLEGRO_EVENT *e);
	void checkKeyboard(ALLEGRO_EVENT *e);
public:
	EventManager(ALLEGRO_DISPLAY*, ALLEGRO_EVENT_QUEUE*);
	~EventManager();
	void init();
	void checkEvents();


};

class Engine
{
	
public:
	Engine();
	~Engine();
	void init();
	void run();
};

class Cell {
public:
	Cell(unsigned x_, unsigned y_);
	Cell();
	bool isRoad() { return road; };
	void show();
	void setRoad(bool sign) { road = sign; };
	bool isInside(unsigned x, unsigned y);
private:
	unsigned x, y, x1, y1, x2, y2;
	bool road;
};
#endif
