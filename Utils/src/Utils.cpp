#include <Utils/Utils.h>

//assume that the gravity is in the y-direction (this can easily be changed if need be), and this value gives its magnitude. 
double Globals::g = -9.8;
//this is the direction of the up-vector
V3D Globals::worldUp = V3D(0, 1, 0);
//and the ground plane
Plane Globals::groundPlane = Plane(P3D(0,0,0), Globals::worldUp);
//this is the total ellapsed sim time
double Globals::currentSimulationTime = 0;

// given a list of keywords that map strings to integer values denoting keyword types, this method will determine the type of command that is passed in
int getLineType(char* &line, KeyWord* keywords, int nKeywords){
	for (int i = 0;i < nKeywords;i++) {
		if (strncmp(line, keywords[i].keyWord, strlen(keywords[i].keyWord)) == 0 && isWhiteSpace(line[strlen(keywords[i].keyWord)])) {
			line += strlen(keywords[i].keyWord);
			return keywords[i].retVal;
		}
	}
	return -1;
}

// given a list of keywords that map strings to integer values denoting keyword types, this method will determine the string corresponding to the token passed in
char* getKeyword(int lineType, KeyWord* keywords, int nKeywords) {
	for (int i = 0;i<nKeywords;i++) {
		if (lineType == keywords[i].retVal)
			return keywords[i].keyWord;
	}

	return NULL;
}

bool StringExtensionHas(const char* _csName, const char* _ext)
{
	std::string fileName(_csName);
	std::string fNameExt = fileName.substr(fileName.find_last_of('.') + 1);

	return (fNameExt.compare(_ext) == 0);
}


// *************************************************************************************************************************
/**
*	Skip comment lines and empty lines
*	Returns a string containing the line.
*/
std::string readValidLine(std::ifstream& stream) {
	std::string line;
	while (std::getline(stream, line) && (line[0] == '#' || line[0] == '\0')) {
	}
	return line;
}

/**
*	Skip comment lines and empty lines
*	Returns a string stream containing the line.
*/
std::istringstream readValidLineAsStream(std::ifstream& stream) {
	return std::istringstream(readValidLine(stream));
}

std::string boolToString(bool boolValue) {
	return (boolValue ? "True" : "False");
}

bool stringToBool(std::string boolString) {
	if (boolString == "True") {
		return true;
	} else if (boolString == "False") {
		return false;
	} else {
		throw("String does not represent a bool!!");
	}
}
