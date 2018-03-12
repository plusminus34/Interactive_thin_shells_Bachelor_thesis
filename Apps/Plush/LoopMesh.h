#pragma once 
#include <PlushHelpers/helpers_star.h>
#include <PlushHelpers/triangle.h>
#include "Handler.h"

class LoopMesh : public Handler {

public:
	bool SHIFTED = false;
	void add_sketch(int N);

public:
	enum Mode { MeshMode, TendonMode };
	Mode mode = MeshMode;
	// --
	P3D *find_min_node(vector<P3D *>);
	vector<P3D *> find_min_edge(vector<vector<P3D *>>);
	vector<P3D *> *find_min_sketch(vector<vector<P3D *> *>);
	// --
	P3D *selected_node = nullptr;
	P3D *selected_knot = nullptr;
	vector<P3D *> *selected_sketch = nullptr; // Shift-Click to select a sketch, color differently.
	// --
	Ray ray;
	P3D xy0 = P3D();
	// --
	double POINT_THRESH = .1;
	double EDGE_THRESH = .1;

public:

	vector<P3D *> boundary;
	vector<vector<P3D *>> boundary_edges(); 
	void split_into(vector<P3D *>, vector<P3D *> &);
	// --
	vector<vector<P3D *> *> sketches; 
	vector<vector<P3D *>> sketch_edges(vector<P3D *>); 
	// --
	void bind_knots();
	vector<vector<pair<vector<int>, vector<double>>>> tendon_bindings;

	vector<P3D> triangulated_vertices;
	vector<vector<int>> triangulated_triangles;
	double TRIANGLE_MAX_AREA = .005;

	LoopMesh();
	~LoopMesh();

	void sloppy_symmetrize();
	void draw();
	void perform_triangulation(struct triangulateio &in, struct triangulateio &out);
	void store_triangulation(struct triangulateio &in, struct triangulateio &out);
	void cleanup_triangulation(struct triangulateio &in, struct triangulateio &out);

	void fprintf_info_tup(FILE* fp);
	void dump_v(const char* fName);
	void dump_tri(const char* fName);
	void dump_tdn(const char * fName);
	void dump(const char* fName);

	static vector<vector<int>> loop_edges(const vector<P3D> &points);

public:
	bool mouse_move_(double xPos, double yPos);
	bool mouse_button_(int button, int action, int mods, double xPos, double yPos);
	bool mouse_wheel_(double xOffset, double yOffset);
	bool key_event_(int key, int action, int mods);

public: 
	int ADD_TDN_i = 0;
	bool TRIANGULATE = false;
	bool DRAW_MIRROR = false;

};


	// TODO: Lets make a simplified version of this that just has a boundary.
	// (As well as tendons, I really just mean no holes.)
	// This should also be a handler.
	// Stuff is stored as vector<P3D *>'s
	// We have a P3D *selected_node;
	// We have a P3D *selected_sketch; // (tendon sketch)
	// Bind knots should use the bary 2D variant