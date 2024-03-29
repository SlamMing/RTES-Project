﻿/*
	PROGETTO REAL TIME EMBEDDED SYSTEMS #3
	Car​ ​race.​ ​Simulate​ ​a​ ​car​ ​race​ ​with​ ​N​ ​cars​ ​on​ ​a​ ​circuit​ ​that​ ​can​ ​be​ ​drawn​ ​using​ ​a
	mouse​ ​and​ ​saved​ ​in​ ​a​ ​file.​ ​N-1​ ​cars​ ​must​ ​be​ ​autonomous​ ​(hence​ ​equipped​ ​by
	sensors​ ​that​ ​detect​ ​curves​ ​and​ ​other​ ​cars)​ ​and​ ​one​ ​is​ ​controlled​ ​by​ ​the​ ​player
	through​ ​the​ ​keyboard​ ​arrows.

	HOW TO PLAY:
	1) drawing phase: using the mouse you can drag and draw a road from the starting
	point, making a loop as a track ensures the good functioning of the game; 
	you can press G to save the current track or you can press H and select an existing track file.
	2) racing phase: after finishing the track you can press ENTER and the race will 
	start immediately, you can accelerate or decelerate with respectively W and S 
	while you can turn the car using A for right turn and D for left turn.

	P.S.: it might take a while for the NN to learn how to drive, do not worry if they look
	stuck, they will eventually get over it, usually in under 2 minutes one of the cars completes
	any given track.
*/
#include "GameLoop.h"
#include "pch.h"
#include "Engine.h"
#include <memory>
#include <utility>

using namespace std;
 


int lastLoopTime = clock();

ALLEGRO_DISPLAY		*display;
EventManager		*em;
ALLEGRO_EVENT_QUEUE *queue;
Car					*c;
//AICPtr 					*aicars;
//vector<unique_ptr<AICar>> aicars;
struct gestore_t {
	pthread_t		cars[N];
	//mutex privato per accedere alle risorse di una AICar
	pthread_mutex_t acMutex[N]; 
	//semaforo per la sincronizzazione delle AICar
	sem_t			acPriv[N];
	//mutex per accedere alle risorse del giocatore
	pthread_mutex_t carMutex;
};
gestore_t *gestore;

Cell griglia[WIDTH / CELL_WIDTH][HEIGHT / CELL_WIDTH];
//neuron class
class Neuron {
public:
	Neuron(unsigned nOutputs, unsigned index);
	void setOutput(double val) { output = val; }
	double getOutput() const { return output; }
	void feedForward(const Layer &prevLayer);
	void calcOutputGradients(double targetVal);
	void calcHiddenGradients(const Layer &nextLayer);
	void updateInputWeights(Layer &prevLayer);
	static double eta;
	static double alpha;
	static double randomWeight(void) { return rand() / double(RAND_MAX); }
	static double transferFunction(double x);
	static double transferFunctionDerivative(double x);
	double sumDOW(const Layer &nextLayer) const;
	double output, gradient;
	unsigned mIndex;
	vector<Connection*> outputWeights;

};


//net class
class Net {
public:
	Net(const vector<unsigned> &topology);
	Net(){ return; }
	void feedForward(const vector<double> &inputs);
	void backProp(const vector<double> &targets);
	void getResults(vector<double> &result) const;
	vector<Layer*> lays;
	double error;
	double recentAverageError;
	double recentAverageSmoothingFactor;
};

//****************end decl***********************


double Neuron::eta = 0.1;
double Neuron::alpha = 0.3;

//**************************end decl**************************

void Neuron::updateInputWeights(Layer &prevLayer) {
	//aggiorno i pesi nello struct connection associato ai neuroni
	//del layer precedente

	for (unsigned n = 0; n < prevLayer.size(); n++) {
		Neuron &neuron = *prevLayer[n];
		double oldDeltaWeight = neuron.outputWeights[mIndex]->dWeight;
		double newDeltaWeight =
			eta //learning rate
			* neuron.getOutput()
			* gradient
			+ alpha
			* oldDeltaWeight;
		neuron.outputWeights[mIndex]->dWeight = newDeltaWeight;
		neuron.outputWeights[mIndex]->weight += newDeltaWeight;

	}

}
double Neuron::sumDOW(const Layer &nextLayer) const {
	double sum = 0.0;

	for (unsigned n = 0; n < nextLayer.size() - 1; n++) {
		sum += outputWeights[n]->weight * nextLayer[n]->gradient;
	}
	return sum;
}
void Neuron::calcHiddenGradients(const Layer &nextLayer) {
	double dow = sumDOW(nextLayer);
	gradient = dow * Neuron::transferFunctionDerivative(output);
}
void Neuron::calcOutputGradients(double targetVal) {

	double delta = targetVal - output;
	gradient = delta * Neuron::transferFunctionDerivative(output);
}

double Neuron::transferFunction(double x) {
	if (x < 0)
		return 0;
	return x;
}
double Neuron::transferFunctionDerivative(double x) {
	if (x < 0)
		return 0;
	return 1;
}
void Neuron::feedForward(const Layer &prevLayer) {

	double sum = 0.0;

	for (unsigned n = 0; n < prevLayer.size(); n++) {
		sum += prevLayer[n]->getOutput() * prevLayer[n]->outputWeights[mIndex]->weight;
	}
	
	output = transferFunction(sum);
}
Neuron::Neuron(unsigned nOutputs, unsigned index) {
	mIndex = index;
	for (unsigned c = 0; c < nOutputs; c++) {
		outputWeights.push_back(new Connection());
		outputWeights.back()->weight = randomWeight();
	}

}


void Net::getResults(vector<double> &result) const {
	result.clear();

	for (unsigned n = 0; n < lays.back()->size() - 1; n++) {
		result.push_back((*lays.back())[n]->getOutput());
	}
}
void Net::backProp(const vector<double> &targets) {
	//calcola root mean square error
	Layer &outputLayer = *lays.back();
	error = 0.0;

	for (unsigned n = 0; n < outputLayer.size() - 1; n++) {
		double delta = targets[n] - outputLayer[n]->getOutput();
		error += delta * delta;
	}
	error /= outputLayer.size() - 1;
	error = sqrt(error);

	//performance

	recentAverageError = (recentAverageError * recentAverageSmoothingFactor + error)
		/ (recentAverageSmoothingFactor + 1.0);
	//calcola gradiente del layer output 
	for (unsigned n = 0; n < outputLayer.size() - 1; n++) {
		outputLayer[n]->calcOutputGradients(targets[n]);
	}
	//calcola gradiente dei hidden layers
	for (unsigned nLayer = lays.size() - 2; nLayer > 0; --nLayer) {
		Layer &hiddenLayer = *lays[nLayer];
		Layer &nextLayer = *lays[nLayer + 1];

		for (unsigned n = 0; n < hiddenLayer.size(); ++n) {
			hiddenLayer[n]->calcHiddenGradients(nextLayer);
		}
	}
	//per tutti i layer da output al primo hidden
	//update di tutti i pesi
	for (unsigned nLayer = lays.size() - 1; nLayer > 0; --nLayer) {
		Layer &l = *lays[nLayer];
		Layer &prevLayer = *lays[nLayer - 1];

		for (unsigned n = 0; n < l.size() - 1; n++) {
			l[n]->updateInputWeights(prevLayer);
		}
	}
}
void Net::feedForward(const vector<double> &inputs) {

	//set input to input neurons
	for (unsigned i = 0; i < inputs.size(); i++) {
		(*lays[0])[i]->setOutput(inputs[i]);
	}

	//propaga
	for (unsigned nLayer = 1; nLayer < lays.size(); ++nLayer) {
		Layer &prevLayer = *lays[nLayer - 1];
		for (unsigned n = 0; n < lays[nLayer]->size() - 1; n++) {
			(*lays[nLayer])[n]->feedForward(prevLayer);
		}

	}

}
Net::Net(const vector<unsigned> &topology) {
	unsigned nLayers = topology.size();
	//creazione layers
	for (unsigned layerNum = 0; layerNum < nLayers; ++layerNum) {
		lays.push_back(new Layer());
		unsigned nOutputs = layerNum == topology.size() - 1 ? 0 : topology[layerNum + 1];

		//aggiungo neuroni al layer
		for (unsigned neuronNum = 0; neuronNum <= topology[layerNum]; ++neuronNum) {
			lays.back()->push_back(new Neuron(nOutputs, neuronNum));
		}
		lays.back()->back()->setOutput(1.0);
	}
	//setto i bias
	
	
}


template<typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
	return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}
//#####################CAR#######################

Car::Car(float px, float py)
{
	x = px;
	y = py;

	pts = new float[8];

	updatePts();
	color = al_map_rgb(rand() % 255, rand() % 255, rand() % 255);
}


Car::~Car()
{
}
void Car::move() {
	sx = vel * cos(angle);
	sy = vel * sin(angle);

	if (x + sx < WIDTH && x + sx > 0)
		x += sx;
	if (y + sy < HEIGHT && y + sy > 0)
		y += sy;
}
void Car::tick() {
	move();
	//attrito
	vel *= 0.995;
	updatePts();
}
void Car::show() {
	
	al_draw_filled_polygon(pts, 4, color);
}

void Car::turn(int dir) {
	angle += dir * dAngle;
}

void Car::accelerate(int dir) {
	if (abs(vel) <= MAXSPEED)
		vel += dir * dAcc;
}
//calcola gli spigoli della macchina usando l'angolo della macchina e il punto centrale
void Car::updatePts() {
	float points[] = {
	(float)x + cos(210 * M_PI / 180.0 + angle) * VEHICLE_LENGTH, (float)y + sin(210 * M_PI / 180.0 + angle) * VEHICLE_LENGTH,
	(float)x + cos(150 * M_PI / 180.0 + angle) * VEHICLE_LENGTH,(float)y + sin(150 * M_PI / 180.0 + angle) * VEHICLE_LENGTH,
	(float)x + cos(30 * M_PI / 180.0 + angle) * VEHICLE_LENGTH,	(float)y + sin(30 * M_PI / 180.0 + angle) * VEHICLE_LENGTH,
	(float)x + cos(330 * M_PI / 180.0 + angle) * VEHICLE_LENGTH,(float)y + sin(330 * M_PI / 180.0 + angle) * VEHICLE_LENGTH,
	};

	__gnu_cxx::copy_n(points, 8, pts);

}
//resetta la macchina al centro della cella attuale
void Car::reset() {
	unsigned ny, nx;
	nx = getCX();
	ny = getCY();
	nx = nx * CELL_WIDTH + CELL_WIDTH / 2;
	ny = ny * CELL_WIDTH + CELL_WIDTH / 2;
	x = nx;
	y = ny;
	vel = 0;

}
//funzioni per calcolare le coordinate della cella in cui si trova la macchina
unsigned Car::getCX() {
	unsigned temp = floor(x / CELL_WIDTH);
	if (temp == 0) return 1;
	else if (temp == nWidth) return (nWidth - 1);
	else return temp;
}

unsigned Car::getCY() {
	unsigned temp = floor(y / CELL_WIDTH);
	if (temp == 0) return 1;
	else if (temp == nHeight) return (nHeight - 1);
	else return temp;
}

double Car::getX() {
	return (pts[0] + pts[4]) / 2;
}

double Car::getY() {
	return (pts[1] + pts[5]) / 2;
}

double Car::getAngle() {
	return angle;
}
//#############endCar###########################

class AICar : public Car {


public:
	void step(int index);
	void die();
	AICar(float, float);
	AICar() { Car(); return; }
	void initNet();
	void initSensor();
	Sensor **sensori;
	double ins[5];
	Net *net;
	
};

//#############AICAR ############################

AICar::AICar(float x, float y) : Car(x, y) {

	vel = MAXSPEED;

	
}
void AICar::initSensor(){
	//-------------------------------------------------
	//inizializzo sensori di prossimità a certi angoli
	//questi mi daranno gli input per la rete neurale
	//-------------------------------------------------
	int j = 0;
	sensori = new SensPtr[5];
	for (double i = M_PI / 2; i >= -M_PI / 2; i -= M_PI / 4) {
		sensori[j] = new Sensor(x, y, i);
		ins[j] = 0;
		j++;
	}
}
void AICar::initNet(){

	//rete neurale 5(input) -> 3 (hidden) -> 2 (output)
	std::vector<unsigned> topology;
	topology.push_back(5);
	topology.push_back(3);
	topology.push_back(2);
	this->net = new Net(topology);
}

void AICar::step(int index) {
//ricevi input dai sensori
	vector<double> inputs;
	for (int i = 0; i < 5; i ++) {
		sensori[i]->update(x, y, getAngle());
		ins[i] = sensori[i]->getDistance(x, y)/SENSOR_RANGE;
		inputs.push_back(ins[i]);

	}
//feeda gli input alla rete
	net->feedForward(inputs);
	

//propaga e prendi i risultati
	std::vector<double> res;
	net->getResults(res);
//usa i risultati per girare la macchina di un multiplo di 6°
	angle += ((res[0] > res[1]) ? res[0] : -res[1])*M_PI / 30;

//assicuro mutua esclusione su mutex privato per muovere la macchina 
	pthread_mutex_lock(&gestore->acMutex[index]);
	move();
	pthread_mutex_unlock(&gestore->acMutex[index]);

	updatePts();
	
}

void AICar::die() {
//reset posizione
	x = spawnX;
	y = spawnY;
	angle = 0;
	vector<double> targets;
	double sx = (ins[0] + ins[1])/2;
	double dx = (ins[3] + ins[4])/2;
//training
	targets.push_back(sx);
	targets.push_back(dx);
	net->backProp(targets);
}
//distanza tra due punti
double dist(point p1, point p2) {
	return sqrt(pow(p1.x - p2.x , 2) + pow(p1.y - p2.y , 2));
}
//intersezione tra due linee o 4 punti
point intersezione(line l1, line l2) {
	double detA = l1.a * l2.b - l2.a * l1.b;
	double detX = l2.b * l1.c - l1.b * l2.c;
	double detY = l1.a * l2.c - l2.a * l1.c;
	point p(detX / detA, detY / detA);
	return p;
}
point intersezione(point a, point b, point c, point d) {
	point r = b - a;
	point s = d - c;
	double de = r.x * s.y - r.y * s.x;
	double u = ((c.x - a.x) * r.y - (c.y - a.y) * r.x) / de;
	double t = ((c.x - a.x) * s.y - (c.y - a.y) * s.x) / de;

	if (u >= 0 && u <= 1 && t >= 0 && t <= 1) {
		return a + r * t;
	}
	else return point(-1, -1);

}
//#################Sensor##########################
Sensor::Sensor(float x, float y, double angle) : angle_(angle), lenght(SENSOR_RANGE), x_(x), y_(y) {}
//ruota il sensore del suo angolo più quello della macchina
void Sensor::update(float x, float y, double a) {
	x_ = x + cos(angle_ + a)*lenght;
	y_ = y + sin(angle_ + a)*lenght;
}

double Sensor::getDistance(double cx, double cy) {
	unsigned ci = floor(cx / CELL_WIDTH);
	unsigned cj = floor(cy / CELL_WIDTH);
	double distance = lenght;
	point car(cx, cy);
	point sens(x_, y_);
	point intersection(0, 0);
//-------------------------------------------------------------
// per ogni cella faccio raycasting del sensore su tutte le pareti
// delle celle strada adiacenti a una cella muro, se la distanza trovata
// è minore del range del sensore procedo a salvarlo come nuovo minimo
//--------------------------------------------------------------
	for (unsigned i = ci - 1; i <= ci + 1; i++) {
		for (unsigned j = cj - 1; j <= cj + 1; j++) {
			if (griglia[i][j].isRoad()) {
				//lato superiore
				if (!griglia[i][((j==0) ? j : j - 1)].isRoad() || j == 0) {
						point p1(i*CELL_WIDTH, j*CELL_WIDTH);
						point p2((i + 1)*CELL_WIDTH, j*CELL_WIDTH);
					
						point ix = intersezione(car, sens, p1, p2);
						if (ix.x > 0 && ix.y > 0) {
							double temp = dist(car, ix);
							if (temp < distance) {
								distance = temp;
								intersection = ix;
							}
						}
					}
				//lato inferiore
				if (!griglia[i][((j == nHeight - 1) ? j : j + 1)].isRoad() || j == nHeight - 1) {
					point p1(i*CELL_WIDTH, (j + 1)*CELL_WIDTH);
					point p2((i + 1)*CELL_WIDTH, (j + 1)*CELL_WIDTH);
					point ix = intersezione(car, sens, p1, p2);
					if (ix.x > 0 && ix.y > 0) {
						double temp = dist(car, ix);
						if (temp < distance) {
							distance = temp;
							intersection = ix;
						}
					}
				}
				//lato dx
				if (!griglia[((i == nWidth - 1) ? i : i + 1)][j].isRoad() || i == nWidth - 1) {
					point p1((i + 1)*CELL_WIDTH, j*CELL_WIDTH);
					point p2((i + 1)*CELL_WIDTH, (j + 1)*CELL_WIDTH);
					point ix = intersezione(car, sens, p1, p2);
					if (ix.x > 0 && ix.y > 0) {
						double temp = dist(car, ix);
						if (temp < distance) {
							distance = temp;
							intersection = ix;
						}
					}
				}
				//lato sx
				if (!griglia[((i == 0) ? i : i - 1)][j].isRoad() || i == 0) {
					point p1(i*CELL_WIDTH, j*CELL_WIDTH);
					point p2(i*CELL_WIDTH, (j + 1)*CELL_WIDTH);
					point ix = intersezione(car, sens, p1, p2);
					if (ix.x > 0 && ix.y > 0) {
						double temp = dist(car, ix);
						if (temp < distance) {
							distance = temp;
							intersection = ix;
						}
					}
				}
			}
		}
	}
//----------------------------------------------------
//faccio la stessa cosa per le coordinate del giocatore 
//così imparano a schivare il giocatore
//mutua esclusione per evitare conflitti
//----------------------------------------------------
	pthread_mutex_lock(&gestore->carMutex);
	float * points = c->getPts();
	point p1(points[0], points[1]);
	point p2(points[2], points[3]);
	point p3(points[4], points[5]);
	point p4(points[6], points[7]);
	point ix = intersezione(car, sens, p1, p2);
	if (ix.x > 0 && ix.y > 0) {
		double temp = dist(car, ix);
		if (temp < distance) {
			distance = temp;
			intersection = ix;
		}
	}
	ix = intersezione(car, sens, p2, p3);
	if (ix.x > 0 && ix.y > 0) {
		double temp = dist(car, ix);
		if (temp < distance) {
			distance = temp;
			intersection = ix;
		}
	}
	ix = intersezione(car, sens, p3, p4);
	if (ix.x > 0 && ix.y > 0) {
		double temp = dist(car, ix);
		if (temp < distance) {
			distance = temp;
			intersection = ix;
		}
	}
	ix = intersezione(car, sens, p1, p4);
	if (ix.x > 0 && ix.y > 0) {
		double temp = dist(car, ix);
		if (temp < distance) {
			distance = temp;
			intersection = ix;
		}
	}
	pthread_mutex_unlock(&gestore->carMutex);
	
	return distance;
}

//################endSensor######################

//******************************utility******************************
void setCell(double x, double y, bool sign = true) {
	
	unsigned i = floor(x / CELL_WIDTH);
	unsigned j = floor(y / CELL_WIDTH);
	// se è già strada esco
	if (griglia[i][j].isRoad())
		return;
	if (i == 0 || i == WIDTH / CELL_WIDTH - 1)
		return;
	if (j == 0 || j == HEIGHT / CELL_WIDTH - 1)
		return;
	griglia[i][j].setRoad(sign);
}

bool	runs = false;
bool isRunning() {
	return runs;
}
//******************************end utility******************************

//******************************CELL****************************************
Cell::Cell() :	
	x(0),
	y(0),
	x1(x * CELL_WIDTH),
	y1(y * CELL_WIDTH),
	x2((x + 1) * CELL_WIDTH),
	y2((y + 1) * CELL_WIDTH),
	road(false) {}
Cell::Cell(unsigned x_, unsigned y_) :
	x(x_),
	y(y_),
	x1(x * CELL_WIDTH),
	y1(y * CELL_WIDTH),
	x2((x + 1) * CELL_WIDTH),
	y2((y + 1) * CELL_WIDTH),
	road(false)	{}
void Cell::show() {
	al_draw_filled_rectangle(x*CELL_WIDTH, y*CELL_WIDTH, (x + 1)*CELL_WIDTH, (y + 1)*CELL_WIDTH, al_map_rgb(106, 115, 101));
}
bool Cell::isInside(unsigned px, unsigned py) {
	return ((px <= x2) &&
		(px >= x1) &&
		(py >= y1) &&
		(py <= y2));
}
bool isColliding(float *points) {
	//per ogni spigolo della car
	for (int i = 0; i <8; i++) {
		unsigned x = points[i];
		unsigned y = points[++i];
		//controllo che sia dentro una cella strada
		for (int i = 0; i < WIDTH / CELL_WIDTH; i++)
			for (int j = 0; j < HEIGHT / CELL_WIDTH; j++)
				if(!griglia[i][j].isRoad())
					if (griglia[i][j].isInside(x, y))
						return true;
	}
	return false;
}
void checkCollisions() {
	if (isColliding(c->getPts()))
		c->reset();
}

void drawRoad() {
	for (int i = 0; i < WIDTH / CELL_WIDTH; i++)
		for (int j = 0; j < HEIGHT / CELL_WIDTH; j++)
			if (griglia[i][j].isRoad())
				griglia[i][j].show();
}


//***********************END CELL*********************************

bool exists_file(std::string& name) {
	if (FILE *file = fopen(name.c_str(), "r")) {
		fclose(file);
		return true;
	}
	else {
		return false;
	}
}

/*
load file da dialog box in windows
string openfilename(HWND owner = NULL) {
	OPENFILENAME ofn;
	char fileName[MAX_PATH] = "";
	ZeroMemory(&ofn, sizeof(ofn));
	ofn.lStructSize = sizeof(OPENFILENAME);
	ofn.hwndOwner = owner;
	wchar_t wtext[MAX_PATH];
	std::mbstowcs(wtext, fileName, MAX_PATH);
	ofn.lpstrFile = wtext;
	ofn.nMaxFile = MAX_PATH;
	ofn.Flags = OFN_EXPLORER | OFN_FILEMUSTEXIST | OFN_HIDEREADONLY;
	ofn.lpstrDefExt = L"";
	string fileNameStr;

	if (GetOpenFileName(&ofn)) {
		wstring ws(wtext);
		string str(ws.begin(), ws.end());
		fileNameStr = str;
	}
	return fileNameStr;
}
*/
//versione linux
string openfilename(){
  char filename[1024];
  FILE *f = popen("zenity --file-selection", "r");
  fgets(filename, 1024, f);
  string fn(filename);
	pclose(f);
 fn.pop_back();
  return fn;
}
void loadFile() {
	string fn = openfilename();
	ifstream is(fn.c_str());
	
	if(!is)
	std::cout << "Error while loading file" << endl;
	int cols = WIDTH / CELL_WIDTH,
		rows = HEIGHT / CELL_WIDTH;
	for (int i = 0; i < cols; i++) {
		for (int j = 0; j < rows; j++) {
			char c;
			is >> c;
			if (c == 48) {
				griglia[i][j].setRoad(false);
			}else if(c == 49)
				griglia[i][j].setRoad(true);
		}
	}
	for (int j = 0; j < rows; j++) {
		for (int i = 0; i < cols; i++) {
		
			if(griglia[i][j].isRoad())
				std::cout << "1";
			else
				std::cout << "0";
	}
				std::cout << '\n';
	}
}



void saveFile() {

	//trovo un file name disponibile
	string base = "pista";
	string name = "";
	int i = 0;
	while (true) {
		string index = std::to_string(++i);
		name.clear();
		name.append(base + index + ".txt");
		if (!exists_file(name)) {
			break;
		}
	}
	//salvo il file dividendoli per righe
	ofstream save_file(name);
	if(!save_file)
		std::cout << "file non salvato";
	int cols = WIDTH / CELL_WIDTH,
		rows = HEIGHT / CELL_WIDTH;
	for (int i = 0; i < cols; i++) {
		for (int j = 0; j < rows; j++) {
			if (griglia[i][j].isRoad()) 
					save_file << "1";
			else	save_file << "0";
			
			if (j == rows - 1) save_file << endl;
		}
	}
	save_file.close();
}

AICar **aicars;

//state pattern
class state {
public :
	void virtual tick() { assert(1 == 1); };
};

//stato dove il giocatore disegna la strada
class drawState : public state {
	void tick() {
		al_clear_to_color(al_map_rgb(255, 107, 83));
		drawRoad();
		
		al_flip_display();

	}
};
//stato dove si gareggia contro l'IA
class runState : public state {
	void tick() {
		AICar *temp;
		checkCollisions();
		al_clear_to_color(al_map_rgb(45, 255, 116));
		drawRoad();

		//mutua esclusione per muovere la macchina del giocatore
		pthread_mutex_lock(&gestore->carMutex);
		c->tick();
		c->show();
		pthread_mutex_unlock(&gestore->carMutex);
		for (int i = 0; i < N; i++) {
			temp = aicars[i];
			//mutua esclusione per disegnare le macchine AI senza che esse vengano mosse dai thread
			pthread_mutex_lock(&gestore->acMutex[i]);
			temp->show();
			pthread_mutex_unlock(&gestore->acMutex[i]);

		}
		for (int i = 0; i < N; i++) {
			//semaforo di sincronizzazione,
			//altrimenti i thread possono andare più veloce del thread principale (più di 60 fps)
			sem_post(&gestore->acPriv[i]);
			}
		al_flip_display();

	}
};

state *currentState;
drawState *drawing;
runState *running;

void setState(state *s) {
	currentState = s;
}

int tid[N];


void* thread(void* arg) {
	AICPtr mCar;
	int index = *(int *) arg;
	mCar = aicars[index];

	
	while (true) {
		//do gli input alla rete neurale che mi dice verso che direzione muovermi
		mCar->step(index);
		
		if (isColliding(mCar->getPts()))
			mCar->die();
		//sincronizzazione 
		sem_wait(&gestore->acPriv[index]);
		
	}
}
void startCars(gestore_t *g) {
	pthread_attr_t attrs;
	pthread_attr_init(&attrs);
	for (int i = 0; i < N; i++) {
		tid[i] = i;
		pthread_create(&g->cars[i], &attrs, thread, (void*) &tid[i]);
	}
	pthread_attr_destroy(&attrs);
}
//******************EventManager*********************
EventManager::EventManager(ALLEGRO_DISPLAY *d, ALLEGRO_EVENT_QUEUE *q){
//inizializzazione coda eventi e timer per 60 frame al secondo
	this->queue = q;
	this->disp = d;
}

//destructor
EventManager::~EventManager(){
	al_uninstall_keyboard();
}

void EventManager::checkMouse(ALLEGRO_EVENT *e) {
//check if user closed the display
	if (e->type == ALLEGRO_EVENT_DISPLAY_CLOSE)
		GameLoop::setRunning(false);
	else if (e->type == ALLEGRO_EVENT_MOUSE_BUTTON_DOWN) {
			pressed = true;
	}
	else if (e->type == ALLEGRO_EVENT_MOUSE_BUTTON_UP) {
		pressed = false;
	}
	else if (e->type == ALLEGRO_EVENT_MOUSE_AXES) {
		mouseX = e->mouse.x;
		mouseY = e->mouse.y;
	}
}
void EventManager::checkKeyboard(ALLEGRO_EVENT *e) {
	if (e->type == ALLEGRO_EVENT_KEY_DOWN) {
		if (e->keyboard.keycode == ALLEGRO_KEY_W)
			keys[0] = true;
		if (e->keyboard.keycode == ALLEGRO_KEY_S)
			keys[1] = true;
		if (e->keyboard.keycode == ALLEGRO_KEY_A)
			keys[2] = true;
		if (e->keyboard.keycode == ALLEGRO_KEY_D)
			keys[3] = true;
		if (!isRunning() && e->keyboard.keycode == ALLEGRO_KEY_ENTER) {			
			
			runs = true;
			setState(running);
			//fai partire i thread
			
			startCars(gestore); 
		}
	}
	if (e->type == ALLEGRO_EVENT_KEY_UP) {
		if (e->keyboard.keycode == ALLEGRO_KEY_W)
			keys[0] = false;
		if (e->keyboard.keycode == ALLEGRO_KEY_S)
			keys[1] = false;
		if(e->keyboard.keycode == ALLEGRO_KEY_A)
			keys[2] = false;
		if (e->keyboard.keycode == ALLEGRO_KEY_D)
			keys[3] = false;
		if (e->keyboard.keycode == ALLEGRO_KEY_G)
			saveFile();
		if (e->keyboard.keycode == ALLEGRO_KEY_H)
			loadFile();
	}
}
// controller giocatore
void EventManager::accellerateUp() { c->accelerate(1); }
void EventManager::accellerateDown() { c->accelerate(-1); }
void EventManager::accellerateLeft() { c->turn(1); }
void EventManager::accellerateRight() { c->turn(-1); }
void EventManager::init() {
	//add event listeners
	al_register_event_source(queue, al_get_display_event_source(disp));
	al_install_mouse();
	al_register_event_source(queue, al_get_mouse_event_source());
	al_install_keyboard();
	al_register_event_source(queue, al_get_keyboard_event_source());
}
void EventManager::checkEvents() {
	ALLEGRO_EVENT event;
	if (!al_is_event_queue_empty(queue)) { //check if there is a new event in queue
		al_wait_for_event(queue, &event); //mutex per queue
		checkKeyboard(&event);
		checkMouse(&event);
	
	}
	if (pressed) {
		setCell(mouseX, mouseY);
	}
	if (keys[0]) accellerateUp();

	if (keys[1]) accellerateDown();

	if (keys[2]) accellerateRight();

	if (keys[3]) accellerateLeft();


}

//**************end EventManager*********************


//************************ENGINE********************************
Engine::Engine(){
	//initialize allegro and setup display and game loop
	al_init();
	queue = al_create_event_queue();
	display = al_create_display(WIDTH, HEIGHT);
	al_init_primitives_addon();
	gestore = new gestore_t;
	em = new EventManager(display, queue);
	aicars = new AICPtr[N];
	//macchina giocatore
	c =	new Car(spawnX, spawnY);
	//macchine AI
	for (int i = 0; i < N; i++) {
	
	/*aicars.push_back(make_unique<AICar>(spawnX, spawnY));
	aicars.back().get()->initSensor();
	aicars.back().get()->initNet();
	*/	
	aicars[i] = new AICar(spawnX, spawnY);
	aicars[i]->initSensor();
	aicars[i]->initNet();
	}
	//state pattern
	currentState = new state();
	running = new runState();
	drawing = new drawState();
	setState(drawing);
	
	//random seed
	srand(time(NULL));

	//inizializzazione mutex e semafori
	pthread_mutex_init(&gestore->carMutex, NULL);
	for (int i = 0; i < N; i++) {
		pthread_mutex_init(&gestore->acMutex[i], NULL);
		sem_init(&gestore->acPriv[i], 0, 0);
	}

}


//deconstructor
Engine::~Engine() {
	al_destroy_display(display);
	for (int i = 0; i < N; i++) {
		pthread_join(gestore->cars[i], NULL);
	}
}

	void Engine::init(){
	em->init();
	for (int i = 0; i < WIDTH / CELL_WIDTH; i++)
		for (int j = 0; j < HEIGHT / CELL_WIDTH; j++)
			griglia[i][j] = (Cell(i, j));

	unsigned x = c->x / CELL_WIDTH;
	unsigned y = c->y / CELL_WIDTH;


	griglia[x][y].setRoad(true);
	}

	void Engine::run() {
		em->checkEvents();
		currentState->tick();

	}
	

int main()
{	
	Engine *e;
	e = new Engine;
	e->init();
	
	while (GameLoop::isRunning()) {
		int now = clock();
		
		if (now - lastLoopTime >= OPTIMAL_TIME) {
	
			float fps = 1000000 / (now - lastLoopTime);
			cout << fps << endl;
			lastLoopTime = now;

			e->run();
		}
	}
	
}
