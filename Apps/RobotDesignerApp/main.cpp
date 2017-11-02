
#include "RobotDesignerApp.h"

int main(void){
	unsigned int num_threads = max(atoi(getenv("OMP_NUM_THREADS")), 1);
	omp_set_num_threads(24);

	RobotDesignerApp app;
	app.runMainLoop();

	return 0;
}


