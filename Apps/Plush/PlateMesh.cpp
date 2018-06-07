#include "PlateMesh.h"
#include <PlushHelpers/helpers_star.h>
#include <PlushHelpers/bary.h>
#include "CSTSimulationMesh3D.h"

PlateMesh::PlateMesh(GLCamera *camera, Plane *drag_plane) {
	this->camera = camera;
	this->drag_plane = drag_plane;

	int NUM_TENDONS = 6;
	int NUM_POINTS_PER_TENDON = 20;
	for (int i = 0; i < NUM_TENDONS; ++i) { 
		vector<MagicPoint *> magic_points;
		for (int _ = 0; _ < NUM_POINTS_PER_TENDON; ++_) {
			MagicPoint *magic_point = new MagicPoint(dynamic_cast<GLTrackingCamera *>(camera), this, drag_plane);
			magic_point->SAFE_update_s(magic_point->get_s() + V3D(.2, .02*i, 0.));
			magic_points.push_back(magic_point);
		}
		MagicTendon *magic_tendon = new MagicTendon(dynamic_cast<GLTrackingCamera *>(camera), magic_points);
		tendons.push_back(magic_tendon);
	} 

}

void PlateMesh::draw() {

	if (LOAD) {
		LOAD = false;

		char prefix[128] = "../Apps/Plush/data/poly/spice"; 
		try {
			load(prefix);
			cout << "-> Loaded previous design from disk." << endl;
		} catch (...) {
			error("Failed to load.");
		} 
	} else if (SAVE) {
		SAVE = false;

		char prefix[128] = "../Apps/Plush/data/poly/spice"; 
		try {
			save(prefix);
			cout << "-> Saved current design to disk." << endl;
		} catch (...) {
			error("Failed to save.");
		}

	}

	if (BAKE_TETS) { // !!
		if (!BAKED_TETS) {
			BAKED_TETS = true;
			tetgenio input, output;
			try {
				perform_tetrahedralization(input, output);
				store_tetrahedralization(input, output);
			}
			catch (...) {
				v_out.clear();
				tet_out.clear();
				error("TetGen failed.");
			}
		}
	} else {
		BAKED_TETS = false;
	}

	if (BAKE_TDNS) {
		if (!BAKED_TDNS) {
			BAKED_TDNS = true;
			try {
				bake_tendons();
			} catch (...) {
				error("Bake failed.");
			}
		}
	} else {
		BAKED_TDNS = false;
	}
	
	// -- //

	glPushAttrib(GL_ALL_ATTRIB_BITS); {
		glLineWidth(2);
		glPointSize(4);

		if (DRAW_DESIGNER) {

			// TODO: Handler::DRAW_HANDLER = false
			for (auto &tendon : tendons) {
				tendon->draw();
			}

			if (DRAW_POINTS) {
				set_color(WHITE);
				glBegin(GL_POINTS); {
					glvecP3D(points);
				} glEnd();
			}

			if (DRAW_PLATES) {
				for (auto &plate : plates) {
					plate->draw();
				}
			}
		}

		if (DRAW_BAKE) { 
			/*
			set_color(PUMPKIN);
			glBegin(GL_POINTS); {
				glvecP3D(v_out);
			} glEnd();
			*/
			// --
			set_color(HENN1NK);
			for (auto &tet : tet_out) {
				auto Q = [this, tet](int i) {glP3D(v_out[tet[i]]); };
				// --
				glBegin(GL_LINES); {
					Q(0); Q(1); Q(0); Q(2); Q(0); Q(3); Q(1); Q(2); Q(1); Q(3); Q(2); Q(3);
				} glEnd();
			}

			// TODO: if (DRAW_BAKE_TDNS) {

			for (auto &tdn : tdn_out) {

				vector<P3D> tdn_as_vecP3D;

				for (auto &raw_point : tdn) {
					vector<int>    &i_vec = raw_point.first;
					vector<double> &w_vec = raw_point.second;
					P3D p = P3D();
					for (size_t k = 0; k < i_vec.size(); ++k) {
						int    &i = i_vec[k];
						double &w = w_vec[k];
						// --
						p += v_out[i] * w;
					}
					tdn_as_vecP3D.push_back(p);
				}

				glLineWidth(5);
				glPointSize(10);

				set_color(ORCHID);
				glBegin(GL_POINTS); {
					glvecP3D(tdn_as_vecP3D);
				} glEnd();

				set_color(RATIONALITY);
				glBegin(GL_LINE_STRIP); {
					glvecP3D(tdn_as_vecP3D);
				} glEnd();

			}
		}

		if (DRAW_SIMULATION) { 
			char scratch_prefix[128] = "../Apps/Plush/data/tet/scratch";
 
			if (simMesh == nullptr || PLEASE_REBUILD_SIMULATION_MESH) {
				try {
					dump(scratch_prefix);
					cout << "-> Dumped scracth." << endl;
				} catch (...) {
					error("Failed to dump mesh.");
				}

				try {
					PLEASE_REBUILD_SIMULATION_MESH = false;
					// --
					delete simMesh;
					simMesh = new CSTSimulationMesh3D();
					simMesh->spawnSavedMesh(scratch_prefix);
					{
						double min_y = INFINITY;  for (auto &node : simMesh->nodes) { min_y = min(min_y, node->getUndeformedPosition().y()); }
						for (auto &node : simMesh->nodes) { node->setUndeformedPosition(node->getUndeformedPosition() + V3D(0., -min_y)); }
						simMesh->x       = simMesh->X;
						simMesh->x_prime = simMesh->X;
						simMesh->xSolver = simMesh->X;
					}
					simMesh->applyYoungsModulusAndPoissonsRatio(3e4, .25);
					simMesh->addGravityForces(V3D(0., -10., 0.)); // FORNOW
					simMesh->add_contacts_to_boundary_nodes();

					simMesh->DRAW_NODAL_FORCES = false;
					simMesh->DRAW_BOUNDARY = true;
					simMesh->DRAW_BOUNDARY_TRANSPARENT = false;

					cout << "-> Loaded scracth." << endl;

					// delete ik;
					// ik = new SoftIKSolver(simMesh);
					// auto &node = simMesh->nodes[15];
					// ik->toggle_Q(node);
					// node->setTargetPosition(node->getCurrentPosition() + V3D(-.1, .1));
					// ik->SOLVE_DYNAMICS = true; // FORNOW
					// ik->timeStep = dt_sim; // FORNOW

				} catch (...) { 
					error("Failed to load mesh.");
				}

				try {
					queue_plots.clear();
					for (size_t i = 0; i < simMesh->tendons.size(); ++i) {
						P3D KELLY_COLOR = kelly_color(i);
						// --
						auto &tendon = simMesh->tendons[i];
						tendon->SPEC_COLOR = KELLY_COLOR;
						// tendon->SPEC_COLOR = (i < 6) ? GREEN : YELLOW;// GREEKELLY_COLOR;
						// --
						QueuePlot *queue_plot = new QueuePlot(128, -1);
						queue_plots.push_back(queue_plot);
					}
				} catch (...) {
					error("Failed to initialize queue_plots.");
				} 
			}

			simMesh->draw();
			// if (ik != nullptr) { ik->draw(); }
			for (auto &queue_plot : queue_plots) { queue_plot->draw(); }
		} 
	} glPopAttrib();
}

////////////////////////////////////////////////////////////////////////////////
// handler /////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// tetgen //////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void PlateMesh::perform_tetrahedralization(tetgenio &input, tetgenio &output) {
	PLEASE_REBUILD_SIMULATION_MESH = true;

    input.mesh_dim = 3;
    std::vector<double> coor;
    std::vector<int> faces;
    int &n = input.numberofpoints;
    int &f = input.numberoffacets;
	// --
	n = points.size();
	for (auto &point : points) {
		coor.push_back(point[0]);
		coor.push_back(point[1]);
		coor.push_back(point[2]);
	}
	// --
	f = plates.size();
	for (auto &plate : plates) {
		for (auto &i : plate->face_) { // FORNOW: TODO: Does tetgen already support non-triangle plates?
			faces.push_back(i);
		}
	}
	// --
    input.pointlist = new REAL[n * 3];
    input.pointmarkerlist = new int[n];
    //printf("%d %d\n", n, f);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < 3; ++j) {
            input.pointlist[i * 3 + j] = coor[i * 3 + j];
        }
        input.pointmarkerlist[i] = 1;
    }
    input.facetlist = new tetgenio::facet[f];
    input.facetmarkerlist = new int[f];
	int cumm = 0;
    for (int i = 0; i < f; ++i) {
        tetgenio::facet &cur = input.facetlist[i];
        cur.numberofholes = 0;
        cur.holelist = NULL;
        cur.numberofpolygons = 1;
        cur.polygonlist = new tetgenio::polygon[1];
        tetgenio::polygon &p = cur.polygonlist[0];
		p.numberofvertices = plates[i]->face_.size();
        p.vertexlist = new int[p.numberofvertices];
        for (int j = 0; j < p.numberofvertices; ++j) {
            p.vertexlist[j] = faces[cumm + j];
        }
		cumm += p.numberofvertices;
        input.facetmarkerlist[i] = 1;
    }
    tetgenbehavior b;
	char args[128];
	sprintf(args, "Qpq1.414a%f", MAX_TET_VOLUME);
	b.parse_commandline(args);
	tetrahedralize(&b, &input, &output);

}

void PlateMesh::store_tetrahedralization(const tetgenio &input, const tetgenio &output) {
	v_out.clear();
	tet_out.clear();
	// --
    int nodeCount = output.numberofpoints;
    int tetCount = output.numberoftetrahedra;
	// --
    for (int i = 0; i < nodeCount; ++i) {
        double *p = output.pointlist + 3 * i;
		v_out.push_back(P3D(p[0], p[1], p[2]));
    }
	// --
    for (int i = 0; i < tetCount; ++i) {
		tet_out.push_back({output.tetrahedronlist[i * 4    ],
						   output.tetrahedronlist[i * 4 + 1],
						   output.tetrahedronlist[i * 4 + 2],
						   output.tetrahedronlist[i * 4 + 3]});
    }
}

void PlateMesh::post_process_tetrahedralization() {
	// TOSTRIP: grab_bounding_box(V3D &v_min, V3D &v_max);
	double min_x = INFINITY; double max_x = -INFINITY;
	double min_y = INFINITY; double max_y = -INFINITY;
	double min_z = INFINITY; double max_z = -INFINITY;
	for (auto &p : v_out) {
		min_x = min(p[0], min_x); max_x = max(p[0], max_x);
		min_y = min(p[1], min_y); max_y = max(p[1], max_y);
		min_z = min(p[2], min_z); max_z = max(p[2], max_z);
	} 
	V3D Min = V3D(min_x, min_y, min_z);
	V3D Max = V3D(max_x, max_y, max_z);

	V3D L = Max - Min;
	double fac = 1. / L.maxCoeff();
	if (!isnan(fac)) { // NOTE: Sloppy nan guard.
		for (auto &p : v_out) {
			p *= fac;
		}
	}

	V3D Sum = Min + Max;
	V3D Translate = -.5*fac*Sum;
	for (auto &p : v_out) {
		p += Translate;
	}
}

void PlateMesh::bake_tendons() { 
	PLEASE_REBUILD_SIMULATION_MESH = true;
	
	tdn_out.clear();

	if (tet_out.size() == 0 || v_out.size() == 0) {
		error("No tetrahedralization found.");
		return;
	}

	// NOTE: Idea is to just take whatever we can.
	vector<MagicTendon *> preprocessed_tendons;

	// NOTE: This leaks memory like crazy.
	for (auto &tendon : tendons) {
		vector<MagicPoint *> add_points;

		for (auto &point : tendon->points) {
			if (!(point->ON_MESH)) {
				// error("Some points not on mesh.");
				continue;
			} else {
				add_points.push_back(point);
			}
		}
		// --
		MagicTendon *add_tendon = new MagicTendon(tendon->camera,add_points);
		if (add_tendon->points.size() >= 2) {
			preprocessed_tendons.push_back(add_tendon);
		}
	}

	for (auto &tendon : preprocessed_tendons) {
		vector<RawPoint> tdn;
		for (auto &point : tendon->points) {
			P3D s = point->get_s();

			double min_d = INFINITY;
			PlateMesh::RawPoint min_rawpoint = { {}, {} };
			bool FOUND_A_VALID_TRIANGLE = false;

			for (auto &tet : tet_out) {
				int i0 = tet[0];
				int i1 = tet[1];
				int i2 = tet[2];
				int i3 = tet[3];
				// --
				vector<int> f0_ = { i0, i1, i2 };
				vector<int> f1_ = { i0, i2, i3 };
				vector<int> f2_ = { i0, i3, i1 };
				vector<int> f3_ = { i1, i2, i3 };
				// --
				for (auto &f_ : {f0_, f1_, f2_, f3_}) {
					vector<P3D> f = { v_out[f_[0]], v_out[f_[1]], v_out[f_[2]] };
					// --
					// BEG TODO STRIP
					Plane p = Plane(f[0], f[1], f[2]);
					P3D q = p.getProjectionOf(s);
					// -- 
					V3D n = p.n; 
					V3D u, v;
					n.getOrthogonalVectors(u, v); 
					P3D o = (f[0] + f[1] + f[2]) / 3.;
					Frame frame = Frame({ u, v, n }, o);
					// --
					vector<P3D> f_prime = { frame.world2local(f[0]), frame.world2local(f[1]), frame.world2local(f[2])};
					P3D q_prime =           frame.world2local(q); 
					P3D lambda = get_bary_lambda_2d(f_prime, q_prime); // TODO: Error guard
					// END TODO STRIP

					if (lambda_inside_triangle(lambda)) {
						FOUND_A_VALID_TRIANGLE = true;
						double d = abs(p.getSignedDistanceToPoint(s));
						if (d < min_d) {
							min_d = d;
							vector<int>    i_out = f_;
							vector<double> w_out = { clamp01(lambda[0]), clamp01(lambda[1]), clamp01(lambda[2]) };
							min_rawpoint = RawPoint(i_out, w_out);
						}
					}
					
				}
			} 
			if (!FOUND_A_VALID_TRIANGLE) {
				error("Could not project.");
			} else {
				tdn.push_back(min_rawpoint);
			}
		} 
		tdn_out.push_back(tdn);
	} 
}

void PlateMesh::save_s(const char *fName) {
	FILE* fp = fopen(fName, "w"); {
		;
	} fclose(fp);
}

void PlateMesh::save_poly(const char *fName) { 
	FILE* fp = fopen(fName, "w"); {
		;
	} fclose(fp);
}

void PlateMesh::save_plan(const char *fName) {
	FILE* fp = fopen(fName, "w"); {
		for (auto &tendon : tendons) {
			for (auto &point : tendon->points) {
				P3D s = (point->ON_MESH) ? point->get_s() : point->s_prime_prev_;
				int i = (point->PLATE != nullptr) ? find_index(plates, point->PLATE) : -1;
				fprintf(fp, "%lf %lf %lf %d ", s[0], s[1], s[2], i);
			}
			fprintf(fp, "\n");
		}
	} fclose(fp);
}

void PlateMesh::save(const char *prefix) { 
	char sName[128];
	apply_suffix(prefix, "s", sName);
	cout << "saving " << sName << "..." << endl;
	save_s(sName);

	char polyName[128];
	apply_suffix(prefix, "poly", polyName);
	cout << "saving " << polyName << "..." << endl;
	save_poly(polyName); 

	char planName[128];
	apply_suffix(prefix, "plan", planName);
	cout << "saving " << planName << "..." << endl;
	save_plan(planName); 
}

void PlateMesh::load_s(const char *fName) {
	FILE* fp = fopen(fName, "r"); {
		;
	} fclose(fp);
}

void PlateMesh::load_poly(const char *fName) { 
	FILE* fp = fopen(fName, "r"); {
		;
	} fclose(fp);
}

void PlateMesh::load_plan(const char *fName) {

	// dynamic_cast<GLTrackingCamera *>(camera)->rotAboutRightAxis = 0.;
	// dynamic_cast<GLTrackingCamera *>(camera)->rotAboutUpAxis = 0.;

	tendons.clear();

	vector<vector<pair<P3D, int>>> tendons_as_vecVecPairP3DInt;
	FILE* fp = fopen(fName, "r"); {
		if (fp != nullptr) {

			P3D s = P3D();
			int plate_i = -1;

			const int LINESZ = 1024;
			char line[LINESZ]; 
			while (fgets(line, LINESZ, fp) != NULL) { 
				vector<pair<P3D, int>> tendon;
				// https://stackoverflow.com/questions/10826953/sscanf-doesnt-move-scans-same-integer-everytime-c 
				int nums_now, bytes_now;
				int bytes_consumed = 0, nums_read = 0;
				while ((nums_now = sscanf(line + bytes_consumed, "%lf %lf %lf %d %n", &s[0], &s[1], &s[2], &plate_i, &bytes_now)) > 0) {
					bytes_consumed += bytes_now; nums_read += nums_now;
					tendon.push_back(make_pair(s, plate_i));
				} 

				tendons_as_vecVecPairP3DInt.push_back(tendon);
			}
		}
	} fclose(fp);

	for (auto &tendon : tendons_as_vecVecPairP3DInt) {
		vector<MagicPoint *> tmp;
		for (auto &raw_point : tendon) {
			P3D s = raw_point.first;
			int plate_i = raw_point.second;
			auto magic_point = new MagicPoint(dynamic_cast<GLTrackingCamera *>(camera), this, drag_plane);
			// --
			if (plate_i == -1) {
				continue;
				magic_point->ON_MESH = false;
				magic_point->PLATE = nullptr;
				magic_point->s_prime_prev_ = s;
				magic_point->enforce_fallback();
			} else {
				magic_point->ON_MESH = true;
				magic_point->PLATE = plates[plate_i];
				magic_point->SAFE_update_s(s);
			}
			tmp.push_back(magic_point);
		} 
		tendons.push_back(new MagicTendon(dynamic_cast<GLTrackingCamera *>(camera), tmp));
	}

	/*
	vector<MagicTendon *> rotated;
	for (auto &tendon : tendons) {
		vector<MagicPoint *> tmp;
		for (auto &point : tendon->points) {
			auto magic_point = new MagicPoint(dynamic_cast<GLTrackingCamera *>(camera), this, drag_plane);
			magic_point->ON_MESH = true;
			magic_point->PLATE = point->PLATE;
			magic_point->SAFE_update_s((P3D)rotateVec(point->get_s(), PI / 3., V3D(0., 1., 0.)));
			tmp.push_back(magic_point); 
		}
		rotated.push_back(new MagicTendon(dynamic_cast<GLTrackingCamera *>(camera), tmp)); 
	} 
	for (auto &tdn : rotated) { tendons.push_back(tdn); }
	*/

	// TODO: Strip these into buttons
	if (false) {

		auto z_mirror = [](vector<MagicTendon *> &tendons_) {
			vector<MagicTendon *> to_add;
			for (auto &tendon_ : tendons_) {
				vector<MagicPoint *> points;
				for (auto point_ : tendon_->points) {
					MagicPoint *point = new MagicPoint(*point_);
					P3D s = point->get_s();
					point->SAFE_update_s(P3D(s[0], s[1], -s[2]));
					point->rebind();
					points.push_back(point);
				}
				to_add.push_back(new MagicTendon(tendon_->camera, points));
			}
			// -- 
			for (auto &tendon : to_add) { tendons_.push_back(tendon); }
		};

		auto y_mirror = [](vector<MagicTendon *> &tendons_) {
			vector<MagicTendon *> to_add;
			for (auto &tendon_ : tendons_) {
				vector<MagicPoint *> points;
				for (auto point_ : tendon_->points) {
					MagicPoint *point = new MagicPoint(*point_);
					P3D s = point->get_s();
					point->SAFE_update_s(P3D(s[0], -s[1], s[2]));
					point->rebind();
					points.push_back(point);
				}
				to_add.push_back(new MagicTendon(tendon_->camera, points));
			}
			// --
			for (auto &tendon : to_add) { tendons_.push_back(tendon); }
		};

		auto x_mirror = [](vector<MagicTendon *> &tendons_) {
			vector<MagicTendon *> to_add;
			for (auto &tendon_ : tendons_) {
				vector<MagicPoint *> points;
				for (auto point_ : tendon_->points) {
					MagicPoint *point = new MagicPoint(*point_);
					P3D s = point->get_s();
					point->SAFE_update_s(P3D(-s[0], s[1], s[2]));
					point->rebind();
					points.push_back(point);
				}
				to_add.push_back(new MagicTendon(tendon_->camera, points));
			}
			// --
			for (auto &tendon : to_add) { tendons_.push_back(tendon); }
		};

		auto circular_pattern = [](vector<MagicTendon *> &tendons_, uint N) {
			vector<MagicTendon *> to_add;
			for (uint i = 1; i < N; ++i) {
	 			double theta = (double(i) / double(N)) * 2.*PI; // TODO dfrac()

				for (auto &tendon_ : tendons_) {
					vector<MagicPoint *> points;
					for (auto point_ : tendon_->points) {
						MagicPoint *point = new MagicPoint(*point_);
						P3D s = point->get_s();
						point->SAFE_update_s((P3D) rotateVec(s, theta, V3D(0.,1.,0.)));
						point->rebind();
						points.push_back(point);
					}
					to_add.push_back(new MagicTendon(tendon_->camera, points));
				}
				
			} 
			// --
			for (auto &tendon : to_add) { tendons_.push_back(tendon); }
		};

		vector<MagicTendon *> group_1;
		vector<MagicTendon *> group_2;
		vector<MagicTendon *> group_3;
		vector<MagicTendon *> group_4;

		auto &base_tendon = tendons[0];
		auto &side_tendon = tendons[1]; 
		tendons.clear();

		// base_tendon
		{
			vector<MagicTendon *> bottom = { base_tendon };
			z_mirror(bottom);
			circular_pattern(bottom, 6);
			// --
			group_1 = { bottom[0], bottom[1], bottom[4], bottom[5], bottom[8], bottom[9] };
			group_3 = { bottom[2], bottom[3], bottom[6], bottom[7], bottom[10], bottom[11] };
		}


		// side_tendon
		{
			vector<MagicTendon *> side = { side_tendon };
			y_mirror(side);
			vector<MagicTendon *> dummy = side;
			z_mirror(dummy);
			dummy = { dummy[2], dummy[3] };
			circular_pattern(dummy, 3);
			for (size_t i = 2; i < dummy.size(); ++i) {
				side.push_back(dummy[i]);
			}
			x_mirror(side);
			// --
			group_2 = { side[0], side[1], side[2], side[3], side[4], side[5] };
			group_4 = { side[6], side[7], side[8], side[9], side[10], side[11] };
		}

		for (auto &tdn : group_1) { tendons.push_back(tdn); }
		// for (auto &tdn : group_2) { tendons.push_back(tdn); }
		for (auto &tdn : group_3) { tendons.push_back(tdn); }
		// for (auto &tdn : group_4) { tendons.push_back(tdn); } 

	}

}

void PlateMesh::load(const char *prefix) { 
	// cout << "loading spice.s..." << endl;
	// char sName[128];
	// apply_suffix(prefix, "s", sName);
	// load_s(sName);
	// cout << "Loaded s." << endl;

	// cout << "loading spice.poly..." << endl;
	// char polyName[128];
	// apply_suffix(prefix, "poly", polyName);
	// load_poly(polyName); 
	// cout << "Loaded poly." << endl;

	char planName[128];
	apply_suffix(prefix, "plan", planName);
	cout << "loading " << planName << endl;
	load_plan(planName); 
	cout << "Loaded plan." << endl;
}

void PlateMesh::dump_v(const char* fName) {
	FILE* fp = fopen(fName, "w"); 
	for (auto &p : v_out) { fprintf(fp, "%lf %lf %lf\n", p[0], p[1], p[2]); } 
	fclose(fp);
}

void PlateMesh::dump_tet(const char* fName) {
	FILE* fp = fopen(fName, "w"); 
	for (auto &tet : tet_out) { fprintf(fp, "%d %d %d %d\n", tet[0], tet[1], tet[2], tet[3]); } 
	fclose(fp);
}

void PlateMesh::dump_tdn(const char *fName) {
	FILE* fp = fopen(fName, "w"); 
	for (size_t k1 = 0; k1 < tdn_out.size(); ++k1) {
		auto &tdn = tdn_out[k1];
		for (size_t k2 = 0; k2 < tdn.size(); ++k2) {
			auto &p = tdn[k2];
			// --
			vector<int> &i = p.first;
			vector<double> &w = p.second;
			fprintf(fp, "%d %lf %d %lf %d %lf -1 0.", i[0], w[0], i[1], w[1], i[2], w[2]); // (*)
			// --
			if (k2 != tdn.size() - 1) { fprintf(fp, " "); }
		}
		if (k1 != tdn_out.size() - 1) { fprintf(fp, "\n"); }
	} 
	fclose(fp); 
}

void PlateMesh::dump(const char* prefix) {

	// BEG TOWRAP: bake_tet
	tetgenio input, output;
	perform_tetrahedralization(input, output);
	store_tetrahedralization(input, output); 
	bake_tendons(); // TODO: rename bake_tdn 
	// END TOWRAP

	// post_process_tetrahedralization();
 
	cout << "dumping spice.X..." << endl;
	char vName[128];
	apply_suffix(prefix, "X", vName);
	dump_v(vName);

	cout << "dumping spice.tet..." << endl;
	char tetName[128];
	apply_suffix(prefix, "tet", tetName);
	dump_tet(tetName); 

	cout << "dumping spice.tdn..." << endl;
	char tdnName[128];
	apply_suffix(prefix, "tdn", tdnName);
	dump_tdn(tdnName); 
} 

// TODO: Make it clear when we're rebuilding the sim mesh versus resetting the simulation...problem?
void PlateMesh::reset_simulation() {
	t_simulation = 0.;
	t_motionplan = 0.;
	// --
	simMesh->energyFunction->initialize(simMesh);
	// --
	simMesh->x       = simMesh->X;
	simMesh->x_prime = simMesh->X;
	simMesh->xSolver = simMesh->X;
	// --
	for (auto &tendon : simMesh->tendons) {
		tendon->set_alphac(0.);
	} 
}

void PlateMesh::step_simulation() {
	for (auto &contact : simMesh->contacts) { contact->update(simMesh->x); }

		double dt_simulation_ = SIMULATION_SPEED_*nominalTimeStep;
		this->simMesh->timeStep = dt_simulation_; // FORNOW
		double dt_motionplan_ = MOTIONPLAN_SPEED_*nominalTimeStep;

		t_simulation += dt_simulation_;
		t_motionplan += dt_motionplan_;
		double m01_t_motionplan = mod01(t_motionplan); 

	// SmoothestStep SS_quiet_start = SmoothestStep(.5*MOTIONPLAN_SPEED_);
	// double quiet_start = SS_quiet_start.g(t_motionplan - .5*MOTIONPLAN_SPEED_*(*SS_quiet_start.eps_ptr));
	double quiet_start = 1.;

	if (true) {
		for (size_t i = 0; i < simMesh->tendons.size(); ++i) { 

			auto &tendon = simMesh->tendons[i];

			int j = int(i / 6);

			auto S = [&](double t_trigger) {
				double signal01 = 0.;
				SmoothestStep SS = SmoothestStep(.1);//.025);
				for (auto &offset : { 0., 1. }) { // (***)
					signal01 += SS.g(m01_t_motionplan - (t_trigger + offset));
				}
				return signal01;
			};

			double signal01;
			if (j == 0) {
				signal01 = 0. + S(2./8.) - S(7./8.);
			} else if (j == 1) {
				signal01 = 0. + S(1./8.) - S(4./8.);
			} else if (j == 2) {
				signal01 = 1. - S(3./8.) + S(6./8.);
			} else if (j == 3) {
				signal01 = 1. + S(5./8.) - S(0./8.);
			}

			double ceiling = .6;

			double alphac = quiet_start*signal01*ceiling;

			tendon->set_alphac_over_alphaz(alphac); 
			tendon->SPEC_COLOR = kelly_color(j);

			queue_plots[i]->add_new_data_point(alphac);
			queue_plots[i]->SPEC_COLOR = kelly_color(j);

		}

		if (ACTUALLY_SOLVE) {
			simMesh->xvPair_INTO_Mesh((SOLVE_DYNAMICS) ? simMesh->solve_dynamics() : simMesh->solve_statics());
		}

	}

	if (false) {
		for (size_t i = 0; i < simMesh->tendons.size(); ++i) { 

			auto &tendon = simMesh->tendons[i];

			int j = (i < 6) ? 0 : 1;

			auto S = [&](double t_trigger) {
				SmoothestStep SS = SmoothestStep(.1);
				return SS.g(m01_t_motionplan - t_trigger) + SS.g(m01_t_motionplan - (t_trigger + 1.)); // (**)
			};

			double signal01;
			if (j == 0) {
				signal01 = S(.2) - S(.6);
			} else {
				signal01 = S(.0) - S(.3);
			}
			double ceiling = .4;

			double alphac = quiet_start*signal01*ceiling;
			tendon->set_alphac_over_alphaz(alphac);
			queue_plots[i]->add_new_data_point(alphac);

		}

		simMesh->xvPair_INTO_Mesh(simMesh->solve_dynamics());
	}

	if (false) {
		// // TOWRAP
		// ik->SOLVE_DYNAMICS = true; // FORNOW
		// ik->timeStep = dt_sim; // FORNOW
		// for (int _ = 0; _ < 3; ++_) { ik->step(); }
		// simMesh->balphac = ik->RETURN_alphac_a();
		// simMesh->solve_dynamics(ik->timeStep);
	}
}

bool PlateMesh::mouse_button_(int button, int action, int mods, double xPos, double yPos) {
	for (auto &tendon : tendons) {
		if (tendon->mouse_button(button, action, mods, xPos, yPos)) {
			return true;
		}
	} 
	return false;
}

bool PlateMesh::mouse_move_(double xPos, double yPos) {
	for (auto &tendon : tendons) {
		if (tendon->mouse_move(xPos, yPos)) {
			return true;
		}
	} 
	return false;
}
