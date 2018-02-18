#include <PlushHelpers\error.h>
#include <Windows.h>
#include <iostream>

bool BERN_ERROR = false;
bool BERN_ERROR_REPORTING = false;
const int N_COOLDOWN = 25;
int BERN_ERROR_COOLDOWN = N_COOLDOWN; 

void error(const std::string &msg) {
	HANDLE hConsole;
	hConsole = GetStdHandle(STD_OUTPUT_HANDLE); 
	SetConsoleTextAttribute(hConsole, 12);
	{
		BERN_ERROR = true;
		std::cout << "[ERROR] " << msg << std::endl;
	}
	SetConsoleTextAttribute(hConsole, 15);
};
