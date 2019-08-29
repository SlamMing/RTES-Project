#include <pthread.h>
#include "pch.h"
#include "Engin.h"

int lastLoopTime = clock();

int main()
{	
	Engine e;
	e.init();
	
	while (GameLoop::isRunning()) {
		int now = clock();
		
		if (now - lastLoopTime >= OPTIMAL_TIME) {
			lastLoopTime = now;

			e.run();
		}
	}
	
}
