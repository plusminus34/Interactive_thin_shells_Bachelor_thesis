#include <conio.h>
#include <stdio.h>
#include <stdlib.h> 
#include <PlushHelpers\error.h>
#include "AppXD.h"
#include "AppEditor2D.h"
#include "AppEditor3D.h"
#include "AppSoftIK.h"
#include "AppSoftLoco.h"

// TODO: PlushApplication sets current direction to your data folder.

int main() {

	const int APP_XD_ID          = 0;
	const int APP_EDITOR2D_ID    = 1;
	const int APP_EDITOR3D_ID    = 2;
	const int APP_IK_ID          = 3;
	const int APP_LOCO_ID        = 4;

	FILE* fp = fopen("../../../Apps/Plush/main.txt", "r");
	int APP = -1;
	if (fp != NULL) {
		fscanf(fp, "%d", &APP);
	} else {
		error("NotFoundError: main.txt");
	}
	if (APP == -1) {
		cout << APP_XD_ID            << " : AppXD\n"
			 << APP_EDITOR2D_ID      << " : AppEditor2D\n"
			 << APP_EDITOR3D_ID      << " : AppEditor3D\n"
			 << APP_IK_ID            << " : AppSoftIK\n"
			 << APP_LOCO_ID          << " : AppSoftLoco\n"
			 << ">> "; 
		APP = getch() - '0';
	} 

	GLApplication* app = NULL;
	switch (APP) {
	case APP_IK_ID:
		app = new AppSoftIK();
		break;
	case APP_EDITOR2D_ID:
		app = new AppEditor2D();
		break;
	case APP_EDITOR3D_ID:
		app = new AppEditor3D();
		break;
	case APP_XD_ID:
		app = new AppXD();
		break;
	case APP_LOCO_ID:
		app = new AppSoftLoco();
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
