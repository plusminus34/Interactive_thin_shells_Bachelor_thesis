#include <Utils/Logger.h>

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <Utils/Utils.h>

std::vector<std::string> Logger::consoleOutput;
int Logger::maxConsoleLineCount = 10;

std::string Logger::ms_strLogPath = std::string("../out");

std::string Logger::ms_strLogFileName = Logger::ms_strLogPath + std::string("\\log.txt");
std::string Logger::ms_strConsoleFileName = Logger::ms_strLogPath + std::string("\\console.txt");

void Logger::consolePrint(const char *fmt, ...) {
	char *pBuffer = NULL;
	GET_STRING_FROM_ARGUMENT_LIST(fmt, pBuffer);

	Path::create(ms_strLogPath);

	// printf("%s", pBuffer);

	static FILE *fp = fopen(ms_strConsoleFileName.c_str(), "wt");
	fprintf(fp, "%s", pBuffer);
	fflush(fp);

	std::vector<std::string> newLines;
	getCharSeparatedStringList(pBuffer, newLines);

	for (uint i = 0; i < newLines.size();i++)
		consoleOutput.push_back(newLines[i]);

	while ((int)consoleOutput.size() > maxConsoleLineCount) consoleOutput.erase(consoleOutput.begin());

	RELEASE_STRING_FROM_ARGUMENT_LIST(pBuffer);
}

void Logger::print(const char *fmt, ...){
	char *pBuffer = NULL;
	GET_STRING_FROM_ARGUMENT_LIST(fmt, pBuffer);

	Path::create(ms_strLogPath);

	printf(pBuffer);

	RELEASE_STRING_FROM_ARGUMENT_LIST(pBuffer);
}

void Logger::logPrint(const char *fmt, ...) {
	char *pBuffer = NULL;
	GET_STRING_FROM_ARGUMENT_LIST(fmt, pBuffer);

	Path::create(ms_strLogPath);

	// printf("%s", pBuffer);

	static FILE *fp = fopen(ms_strLogFileName.c_str(), "wt");
	fprintf(fp, "%s", pBuffer);
	fflush(fp);


	RELEASE_STRING_FROM_ARGUMENT_LIST(pBuffer);
}


#ifdef WIN32
#include <windows.h>
#endif


std::string Filename::isolateFilename(const std::string& _filename)
{

	int iZero = _filename.find_last_of("\\") + 1;
	int iSize = _filename.find_last_of(".") - iZero; //remove extension, whatever it is... 
													// sizeof(.ext) - 1 = 4 - 1 = 3;
	std::string robotName = _filename.substr(iZero, iSize);

	return robotName;
}

/*
	*
*/
bool Path::create(const std::string& _strPath)
{

#ifdef WIN32

	// changed this:
	//std::wstring stemp = std::wstring(_strPath.begin(), _strPath.end());//s.begin(), s.end());
	//LPCWSTR sw = stemp.c_str();

	// ... to this:
	std::string stemp = std::string(_strPath.begin(), _strPath.end());//s.begin(), s.end());
	LPCSTR sw = stemp.c_str();

	bool bSuccess = false;
	if (CreateDirectory(sw, NULL))
	{
		// Directory created
		bSuccess |= true;
	}
	else if (ERROR_ALREADY_EXISTS == GetLastError())
	{
		// Directory already exists
		bSuccess |= true;
	}

	return bSuccess;
#endif

}