#include <conio.h>
#include <stdio.h>
#include <stdlib.h> 
#include <PlushHelpers\error.h>
#include "AppXD.h"
#include "AppSoftIK.h"
#include "AppEditor2D.h"

int main() {

	const int APP_XD_ID          = 0;
	const int APP_IK_ID          = 1;
	const int APP_EDITOR2D_ID    = 2;

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
			 << APP_EDITOR2D_ID      << " : AppEditor2D\n"
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
	case APP_EDITOR2D_ID:
		app = new AppEditor2D();
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
