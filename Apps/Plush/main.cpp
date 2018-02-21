#include <conio.h>
#include <stdio.h>
#include <stdlib.h> 
// --
#include "AppXD.h"
#include "AppSoftIK.h"
#include <PlushHelpers\error.h>

int main() {

	const int APP_XD_ID = 0;
	const int APP_IK_ID = 1;

	FILE* fp = fopen("../../../Apps/Plush/main.txt", "r");
	int APP = -1;
	if (fp != NULL) {
		fscanf(fp, "%d", &APP);
	} else {
		error("NotFoundError: main.txt");
	}
	if (APP == -1) {
		cout << APP_XD_ID            << " : AppXD\n"
			 << APP_IK_ID            << " : AppSoftIK\n"
			 << ">> "; 
		APP = getch() - '0';
	} 

	GLApplication* app = NULL;
	switch (APP) {
	case APP_IK_ID:
		app = new AppSoftIK();
		break;
	case APP_XD_ID:
		app = new AppXD();
		break;
	} 

	if (app != NULL) {
		app->runMainLoop();
		delete app;
		return 0;
	} else {
		error("FatalError");
	}

}
