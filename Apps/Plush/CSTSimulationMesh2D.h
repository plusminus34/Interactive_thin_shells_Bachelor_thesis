#pragma once

#include "SimulationMesh.h"
#include <OptimizationLib/NewtonFunctionMinimizer.h>
#include "CSTElement2D.h"
#include "FixedPointSpring2D.h" 
 
class SimulationMesh;

class CSTSimulationMesh2D : public SimulationMesh {

public:
	CSTSimulationMesh2D();
	~CSTSimulationMesh2D();

// 2D specs
public:
	virtual int D();
	virtual void add_simplices(const vector<vector<int>> &);
	virtual void spawnSimplexMesh();
	virtual void spawnSavedMesh(const char *, bool loadTendons=true);

// stelian impure virtuals
public:
	virtual void setPinnedNode(int ID, const P3D& p);
	virtual int getSelectedNodeID(Ray ray);
};
/*
	virtual int T_I();
    virtual int T_I(int);
	virtual int T_II();
	virtual int T_II(int);
	virtual int W_I();
	virtual int W_II();
 
// loading main
public:
	bool load_mesh(const char* fName);
	void initialize_mesh(const char *fName);

// loading subs
public:
	static void load_v(const char* fName, dVector &z);
	void load_tri(const char* fName);
	void load_tdn(const char* fName);
	void load_winch(const char* fName);
	void seed_winch();
	void post_process_mesh();
	void load_red_pin(const char* fName);

// dumping
public:
	void dump_v(const char* fName, dVector &z);
	void dump_tri(const char* fName);
	void dump_Delta_X(const char* fName);
	void dump_Delta_X2(const char* fName);
	void dump_winch(const char* fName);
	void dump_alpha(const char* fName);
	void dump_red_pin(const char* fName);
	void fprintf_info_tup(FILE *fp); // TODO: remove
 
// buffer work
public:
	vector<NodalTendon2D *> DFS_for_decomposition();
	void clear_buffer();
	void eat_buffer();
	vector<Node *> node_buffer; // nodes go into node buffer
	vector<vector<Node *>> edge_buffer; // once two nodes, they are turned into an edge and pushed into the edge bufffer
	vector<Node *> root_buffer; // the root buffer always has 0 or 1 candidate root nodes

// stelian impure virtuals
public:
	virtual int getSelectedNodeID(Ray ray);
	virtual void setPinnedNode(int ID, const P3D& p);


// zold loading
public:
	void readMeshFromFile(const char* fName);
	static void generateSquareTriMesh(char* fName, double startX=-4.5, double startY=0, double dX=1, double dY=1, int xSize=10, int ySize=10);

};

*/
