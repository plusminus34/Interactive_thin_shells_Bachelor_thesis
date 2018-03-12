#pragma once

#include "PlushApplication.h"
#include "CSTSimulationMesh2D.h"

using namespace Eigen;

class AppArt : public PlushApplication {

public: 
	AppArt();
	inline virtual ~AppArt(void) {}
	inline virtual void restart() {}
	virtual void drawScene();

public:
	CSTSimulationMesh2D *mesh;
	int W() { return getMainWindowWidth(); }
	int H() { return getMainWindowHeight(); }
	float *pixels;

public:
	inline virtual void drawAuxiliarySceneInfo() {}
	virtual void process();

	virtual bool onKeyEvent(int key, int action, int mods);
	virtual bool onCharacterPressedEvent(int key, int mods);
	virtual bool onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos);
	virtual bool onMouseMoveEvent(double xPos, double yPos);
	virtual bool onMouseWheelScrollEvent(double xOffset, double yOffset);
	virtual bool processCommandLine(const std::string& cmdLine);
	
	virtual void saveFile(const char* fName);
	virtual void loadFile(const char* fName);


MatrixXf conv2(const MatrixXf &M, const MatrixXf &K) {

	int R_M = M.rows();
	int C_M = M.cols();
	int R_K = K.rows();
	int C_K = K.cols();

	MatrixXf O;
	O.resize(R_M, C_M);

	for (int row = 0; row < R_M; ++row) { 
		for (int col = 0; col < C_M; ++col) {
			double accumulation = 0;
			double weightsum = 0; 
			for (int  i = 0; i < R_K; ++i) {
				for (int  j = 0; j < C_K; ++j) {
					// TODO: wrap2range()
					int r_M = int(-.5*R_K) + row + i; if (r_M < 0) { r_M += R_M; } if (r_M > R_M - 1) { r_M -= R_M; }
					int c_M = int(-.5*C_K) + col + j; if (c_M < 0) { c_M += C_M; } if (c_M > C_M - 1) { c_M -= C_M; }
					// --
					accumulation += M(r_M, c_M) * K(i, j);
					weightsum += K(i, j);
				} 
			} 

			// O(row,col) = float(accumulation / weightsum);
			if (accumulation > 0.) { O(row, col) = 1.; }
		}
	}
    return O;
}

};



