#pragma once

#include "SimulationMesh.h"

class CSTSimulationMesh3D : public SimulationMesh {

// baiscs
public:
    CSTSimulationMesh3D();
    ~CSTSimulationMesh3D();

// 3D specs
public:
	virtual int D();
	virtual void add_simplices(const vector<vector<int>> &);
	virtual void analyze_lower_simplices();
	virtual void spawnSimplexMesh();
	virtual void spawnSavedMesh(const char *, bool loadTendons=false);

// stelian impure virtuals
    virtual int getSelectedNodeID(Ray ray);
	virtual void setPinnedNode(int ID, const P3D& p);

};
