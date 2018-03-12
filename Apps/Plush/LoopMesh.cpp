#include "LoopMesh.h" 
#include <PlushHelpers/bary.h>
#include <GUILib/GLApplication.h>
#include <GUILib/GLUtils.h>

LoopMesh::LoopMesh() {

	/*
	vector<P3D> hole0;
	int N = 6;
	// for (int i = 0; i < N; ++i) {
		boundary.push_back(P3D(cos(double(i)*2.*PI/double(N)), sin(double(i)*2.*PI/double(N))));
		hole0.push_back(P3D(cos(double(i)*2.*PI/double(N)), sin(double(i)*2.*PI/double(N)))*.5);
	}
	holes.push_back(hole0);
	*/

	boundary.push_back(new P3D(0., 0.));
	boundary.push_back(new P3D(1., 0.));
	boundary.push_back(new P3D(1., 1.));

	// add_sketch(3);
	// add_sketch(5);
}

LoopMesh::~LoopMesh(){ }

void LoopMesh::add_sketch(int N) {
	vector<P3D *> *s0 = new vector<P3D *>();
	for (int i = 0; i < N; ++i) {
		s0->push_back(new P3D(i*.1, i*.1 + .1*ADD_TDN_i));
	}
	sketches.push_back(s0);
	ADD_TDN_i++;
}
 
bool LoopMesh::mouse_move_(double xPos, double yPos) {

	ray = InteractiveWidget::getRayFromScreenCoords(xPos, yPos);
	xy0 = xy0_from_ray(ray);

	if (mode == MeshMode) {
		if (LEFT_CLICKED) {
			if (selected_node != nullptr) {
				*selected_node = xy0;
				return true;
			}
		}
	} else if (mode == TendonMode) {
		if (LEFT_CLICKED) {
			if (selected_knot != nullptr) {
				*selected_knot = xy0;
				return true;
			}
		} 
	}

	return false;
}

P3D *LoopMesh::find_min_node(vector<P3D *> search_nodes) {
	P3D *min_node;
	double min_d = INFINITY;
	for (auto &node : search_nodes) {
		double d = ray.getDistanceToPoint(*node);
		if (d < min_d) {
			min_node = node;
			min_d = d;
		}
	}

	if (min_d < POINT_THRESH) {
		return min_node;
	}
	return nullptr; 
}

vector<P3D *> LoopMesh::find_min_edge(vector<vector<P3D *>> search_edges) {
	vector<P3D *> min_edge = {};
	double min_d = INFINITY;
	for (auto &edge : search_edges) {
		P3D pa = *edge[0];
		P3D pb = *edge[1];
		double d = ray.getDistanceToSegment(pa, pb);
		if (d < min_d) {
			min_edge = edge;
			min_d = d;
		}
	}

	if (min_d < EDGE_THRESH) {
		return min_edge;
	}
	return {};
}

vector<P3D *> *LoopMesh::find_min_sketch(vector<vector<P3D *> *> search_sketches) {
	vector<P3D *> *min_sketch = nullptr;
	double min_d = INFINITY;
	for (auto &sketch : search_sketches) {
		auto edges = sketch_edges(*sketch);
		auto min_edge = find_min_edge(edges);
		if (!min_edge.empty()) {
			P3D pa = *min_edge[0];
			P3D pb = *min_edge[1];
			double d = ray.getDistanceToSegment(pa, pb);
			if (d < min_d) {
				min_sketch = sketch;
				min_d = d;
			}
		}
	}

	return min_sketch;
}

bool LoopMesh::mouse_button_(int button, int action, int mods, double xPos, double yPos) {

	SHIFTED = mods & GLFW_MOD_SHIFT;

	if (button == GLFW_MOUSE_BUTTON_LEFT) {
		if (action == GLFW_PRESS) {
			LEFT_CLICKED = true;
		} else if (action == GLFW_RELEASE) {
			LEFT_CLICKED = false;
		}
	} else if (button == GLFW_MOUSE_BUTTON_RIGHT) {
		if (action == GLFW_PRESS) {
			RIGHT_CLICKED = true;
		} else if (action == GLFW_RELEASE) {
			RIGHT_CLICKED = false;
		}
	}
 
	if (mode == MeshMode) { 
		selected_sketch = nullptr;
		selected_knot = nullptr;

		selected_node = nullptr;
		// ---
		P3D *min_node = find_min_node(boundary); 
		if (min_node != nullptr) {
			if (LEFT_CLICKED) {
				selected_node = min_node;
			} else if (RIGHT_CLICKED) {
				remove_first_instance(boundary, min_node);
			}
			// --
			return true;
		}

		vector<P3D *> min_edge = find_min_edge(boundary_edges());
		if (!min_edge.empty()) {
			if (RIGHT_CLICKED) {
				split_into(min_edge, boundary);
			}
			// --
			return true; 
		}

		return false;

	} else if (mode == TendonMode) { 
		if (LEFT_CLICKED && SHIFTED) {
			auto min_sketch = find_min_sketch(sketches);
			selected_sketch = (min_sketch != selected_sketch) ? min_sketch : nullptr;
			selected_knot = nullptr;
			return true;
		} else if (RIGHT_CLICKED && SHIFTED) { 
			vector<P3D *> *sketch = new vector<P3D *>();
			V3D o = V3D(.1, .1);
			sketch->push_back(new P3D(xy0 + o));
			sketch->push_back(new P3D(xy0 - o));
			sketches.push_back(sketch);
			selected_sketch = sketches.back();
			selected_knot = nullptr;
		} else if (LEFT_CLICKED) {
			selected_knot = (selected_sketch != nullptr) ? find_min_node(*selected_sketch) : nullptr;
		} else if (RIGHT_CLICKED) {
			selected_knot = nullptr;
			// --
			if (selected_sketch != nullptr) {

				auto &search_nodes = *selected_sketch; // (*)
				auto &search_edges = sketch_edges(search_nodes);

				P3D *min_node = find_min_node(search_nodes); 
				if (min_node != nullptr) {
					if (LEFT_CLICKED) {
						selected_node = min_node;
					} else if (RIGHT_CLICKED) {
						if (search_nodes.size() > 2) {
							remove_first_instance(search_nodes, min_node);
						} else {
							remove_first_instance(sketches, selected_sketch); 
							selected_sketch = nullptr;
						}
					}
					// --
					return true;
				}

				vector<P3D *> min_edge = find_min_edge(search_edges);
				if (!min_edge.empty()) {
					if (RIGHT_CLICKED) {
						split_into(min_edge, *selected_sketch);
					}
					// --
					return true; 
				}
			}

		}
		return false;
	} else {
		return false;
	}
}

bool LoopMesh::mouse_wheel_(double xOffset, double yOffset) {

	return false;
}

bool LoopMesh::key_event_(int key, int action, int mods) {

	return false;
}

vector<vector<P3D *>> LoopMesh::sketch_edges(vector<P3D *> sketch) { 
	vector<vector<P3D *>> outer;
	for (int i = 0; i < int(sketch.size()) - 1; ++i) {
		int j = i + 1;
		outer.push_back({ sketch[i], sketch[j] });
	}
	return outer;
}

vector<vector<P3D *>> LoopMesh::boundary_edges() {
	vector<vector<P3D *>> outer;
	for (int i = 0; i < int(boundary.size()); ++i) {
		int j = (i + 1) % int(boundary.size());
		outer.push_back({ boundary[i], boundary[j] });
	}
	return outer;
} 

void LoopMesh::split_into(vector<P3D *> edge, vector<P3D *> &dest) { 
	P3D pa = *edge[0];
	P3D pb = *edge[1];
	Ray v = Ray(pa, (pb - pa).toUnit());
	double t = v.getRayParameterFor(xy0); 
	P3D pc = pa + (pb - pa).toUnit()*t;
	dest.insert(dest.begin() + find_index(dest, edge[0]) + 1, new P3D(pc));
}

void LoopMesh::sloppy_symmetrize() {

	vector<P3D *> right;
	vector<P3D *> left;
	for (auto p : boundary) {
		if ((*p)[0] > 0) {
			right.push_back(p);
		} else {
			left.push_back(p);
		}
	}

	vector<P3D *> candidates = right;
	for (auto p : left) {
		P3D tmp_p = (*p);
		tmp_p[0] *= -1;

		double min_d = 1.;
		int min_i_r = -1;
		int min_i_c = -1;

		int i_c = -1;
		for (auto q : candidates) {
			++i_c;
			auto i_r_ = std::find(right.begin(), right.end(), q);
			int i_r = std::distance(right.begin(), i_r_);
 
			double d = ((tmp_p)-(*q)).norm();
			if (d < min_d) {
				min_d = d;
				min_i_r = i_r;
				min_i_c = i_c;
			}

		}

		if (min_i_r != -1) {
			candidates.erase(candidates.begin() + min_i_c);
			P3D tmp_q = *(right[min_i_r]);
			tmp_q[0] *= -1;
			(*p) = tmp_q;
		}

	}
}

void LoopMesh::draw() {
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glLineWidth(2);
	glPointSize(4);

	set_color(PUMPKIN);
	glBegin(GL_LINE_LOOP);
	for (auto &p : boundary) { glP3D(*p); }
	glEnd();

	set_color(LIGHT_PUMPKIN);
	glBegin(GL_POINTS);
	for (auto &p : boundary) { glP3Dz(*p, 4); }
	glEnd();

	// Warning of points being close
	glBeginBernCircles(YELLOW);
	int N = boundary.size();
	for (int i = 0; i < N; ++i) {
		P3D p = *boundary[i];
		P3D q = *boundary[(i + 1) % N];
		if ((p - q).norm() < 1e-2) {
			glP3Dz(p, 3);
		}
	}
	glEndBernCircles();

	if (TRIANGULATE) {
		struct triangulateio in, out;
		perform_triangulation(in, out);
		store_triangulation(in, out);
		cleanup_triangulation(in, out);

		set_color(HENN1NK);
		for (auto &triangle : triangulated_triangles) {
			glBegin(GL_LINE_LOOP);
			for (auto &i : triangle) {
				glP3Dz(triangulated_vertices[i], -1);
			}
			glEnd();
		}

		set_color(LIGHT_HENN1NK);
		for (auto &triangle : triangulated_triangles) {
			glBegin(GL_TRIANGLES);
			for (auto &i : triangle) {
				glP3Dz(triangulated_vertices[i], -2);
			}
			glEnd();
		}
	}

	glPopAttrib();

	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glLineWidth(4);
	glPointSize(6);
	int i = -1;
	for (auto &sketch : sketches) {
		// P3D color = kelly_color(i++);
		P3D color = (sketch == selected_sketch) ? ORCHID : RATIONALITY;
		set_color(color);
		glBegin(GL_LINE_STRIP);
		for (auto &p : *sketch) { glP3Dz(*p, 1); }
		glEnd();
		set_color(color_swirl(.5, color, WHITE));
		glBegin(GL_POINTS);
		for (auto &p : *sketch) { glP3Dz(*p, 1); }
		glEnd();
	}
	glPopAttrib();
}

////////////////////////////////////////////////////////////////////////////////
// statics /////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

vector<vector<int>> LoopMesh::loop_edges(const vector<P3D>& points) {
	int N = points.size();
	vector<vector<int>> outer;
	for (int i = 0; i < N; ++i) {
		vector<int> inner = { i, (i + 1) % N };
		outer.push_back(inner);
	}
	return outer;
}

////////////////////////////////////////////////////////////////////////////////
// triangle ////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void LoopMesh::bind_knots() { 
	typedef pair<vector<int>, vector<double>> PointBinding;
	typedef vector<PointBinding> TendonBinding;
	tendon_bindings.clear(); 
	for (auto &sketch : sketches) {
		TendonBinding binding;
		for (auto &p_ : *sketch) {
			P3D p = *p_;

			PointBinding min_pb;
			for (auto &tri : triangulated_triangles) { 
				vector<P3D> t = { triangulated_vertices[tri[0]], triangulated_vertices[tri[1]], triangulated_vertices[tri[2]] }; 
				P3D lambda = get_bary_lambda_2d(t, p);
				if (lambda_inside_triangle(lambda)) {
					binding.push_back(PointBinding(tri, {lambda[0], lambda[1], lambda[2]}));
					break;
				}
			}

		}
		if (binding.size() >= 1) {
			tendon_bindings.push_back(binding);
		}
	}
}

void LoopMesh::perform_triangulation(struct triangulateio &in, struct triangulateio &out) {

	int num_holes = 0; 
	int num_boundary_points = boundary.size();
	int num_hole_points = 0;
	int num_total_points = num_boundary_points + num_hole_points;
	int num_total_segments = num_total_points;

	in.numberofpoints = num_total_points;
	in.pointlist = (REAL *)malloc(2 * num_total_points * sizeof(REAL));
	int i_point = 0;
	for (auto &p : boundary) {
		in.pointlist[i_point] = p->x();
		in.pointlist[i_point + 1] = p->y();
		i_point += 2;
	}

	in.numberofpointattributes = 0;
	in.pointattributelist = (double *)NULL;
	in.pointmarkerlist = (int *)NULL;

	in.numberofsegments = num_total_segments;
	in.segmentlist = (int *)malloc(2 * num_total_segments * sizeof(int));
	int i_segment = 0;
	int cumm_offset = 0;
	for (size_t i = 0; i < boundary.size(); ++i) {
		int j = (i + 1) % boundary.size();
		in.segmentlist[i_segment] = i + cumm_offset;
		in.segmentlist[i_segment + 1] = j + cumm_offset;
		i_segment += 2;
	}
	cumm_offset += boundary.size();

	in.segmentmarkerlist = (int *)NULL;

	in.numberofholes = 0;
	in.holelist = (REAL *)malloc(2 * num_holes * sizeof(REAL));

	in.numberofregions = 0;
	in.regionlist = (double *)NULL;

	out.pointlist = (REAL *)NULL;
	out.trianglelist = (int *)NULL;
	out.segmentlist = (int *)NULL;
	out.segmentmarkerlist = (int *)NULL;
	out.edgelist = (int *)NULL;

	char args[128];
	sprintf(args, "pzAeBQa%fq32.5", TRIANGLE_MAX_AREA);
	triangulate(args, &in, &out, (struct triangulateio *)NULL);

};

void LoopMesh::store_triangulation(struct triangulateio &in, struct triangulateio &out) {

	triangulated_vertices.clear();
	triangulated_triangles.clear();

	// FORNOW, area constraint not supported.
	for (int i = 0; i < out.numberofpoints; ++i) {
		double x  = out.pointlist[2*i];
		double y  = out.pointlist[2*i + 1];
		triangulated_vertices.push_back(P3D(x, y));
	}

	for (int i = 0; i < out.numberoftriangles; i++) {
		vector<int> triangle;
		for (int j = 0; j < 3; j++) {
			int k = out.trianglelist[3*i + j];
			triangle.push_back(k);
		}
		triangulated_triangles.push_back(triangle);
	}

}

void LoopMesh::cleanup_triangulation(struct triangulateio &in, struct triangulateio &out) {
		std::free(in.pointlist);
		std::free(in.segmentlist);
		std::free(in.holelist);
		std::free(out.pointlist);
		std::free(out.trianglelist);
		std::free(out.segmentlist);
		std::free(out.edgelist); 
	};

void LoopMesh::fprintf_info_tup(FILE* fp) {
	// info_tup = "num_nodes num_tris"
	int N = triangulated_vertices.size();
	int num_tri = triangulated_triangles.size();
	fprintf(fp, "%d %d\n\n", N, num_tri);
}
 
void LoopMesh::dump_v(const char* fName) {
	FILE* fp = fopen(fName, "w");
	// fprintf_info_tup(fp);

	for (auto &p : triangulated_vertices) {
		fprintf(fp, "%lf %lf\n", p[0], p[1]);
	}

	fclose(fp);
}

void LoopMesh::dump_tri(const char* fName) {
	FILE* fp = fopen(fName, "w");
	// fprintf_info_tup(fp);

	for (auto &triangle : triangulated_triangles) {
		fprintf(fp, "%d %d %d\n", triangle[0], triangle[1], triangle[2]);
	}

	fclose(fp);
}

void LoopMesh::dump_tdn(const char* fName) {
	FILE* fp = fopen(fName, "w");
	// fprintf_info_tup(fp);

	auto &tdn_out = tendon_bindings;
	for (size_t k1 = 0; k1 < tdn_out.size(); ++k1) {
		auto &tdn = tdn_out[k1];
		for (size_t k2 = 0; k2 < tdn.size(); ++k2) {
			auto &p = tdn[k2];
			// --
			vector<int> &i = p.first;
			vector<double> &w = p.second;
			fprintf(fp, "%d %lf %d %lf %d %lf", i[0], w[0], i[1], w[1], i[2], w[2]); // (*)
			// --
			if (k2 != tdn.size() - 1) { fprintf(fp, " "); }
		}
		if (k1 != tdn_out.size() - 1) { fprintf(fp, "\n"); }
	} 

	fclose(fp);
}

void LoopMesh::dump(const char* fName) {

	// FORNOW
	struct triangulateio in, out;
	perform_triangulation(in, out);
	store_triangulation(in, out);
	cleanup_triangulation(in, out);

	// NOTE: call this _after_ running triangulation, but _before processing.
	bind_knots();

	// triangulated_vertices processing block
	// Centers @ O, with max side length unit.
	if (false) { // TODO: Strip, make optional w/ toggle (or better have scaling sliders)
		cout << "processing triangulated_vertices..." << endl; 

		double max_x = -INFINITY;
		double min_x = INFINITY;
		double max_y = -INFINITY;
		double min_y = INFINITY;
		for (auto &p : triangulated_vertices) {
			max_x = std::max(p[0], max_x);
			min_x = std::min(p[0], min_x);
			max_y = std::max(p[1], max_y);
			min_y = std::min(p[1], min_y);
		}

		double x_length = max_x - min_x;
		double y_length = max_y - min_y;
		// NOTE: Sloppy nan guard.
		double fac = 1. / std::max(x_length, y_length);
		if (!isnan(fac)) {
			for (auto &p : triangulated_vertices) {
				p *= fac;
			}
		}

		max_x = -INFINITY;
		min_x = INFINITY;
		max_y = -INFINITY;
		min_y = INFINITY;
		for (auto &p : triangulated_vertices) {
			max_x = std::max(p[0], max_x);
			min_x = std::min(p[0], min_x);
			max_y = std::max(p[1], max_y);
			min_y = std::min(p[1], min_y);
		}
		for (auto &p : triangulated_vertices) {
			// p += V3D(-(min_x + max_x)/2., -min_y);
			p += V3D(-(min_x + max_x) / 2., -(min_y + max_y) / 2.);
		}
	}

	cout << "dumping out.X..." << endl;
	char vName[128];
	apply_suffix(fName, "X", vName);
	dump_v(vName);

	cout << "dumping out.tri..." << endl;
	char triName[128];
	apply_suffix(fName, "tri", triName);
	dump_tri(triName); 

	cout << "dumping out.tdn..." << endl;
	char tdnName[128];
	apply_suffix(fName, "tdn", tdnName);
	dump_tdn(tdnName); 
}
