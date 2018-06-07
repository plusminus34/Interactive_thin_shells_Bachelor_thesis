#include "Plate.h"
#include "PlateMesh.h"
#include <PlushHelpers\triangle.h>
#include <PlushHelpers\bary.h>

Plate::Plate(vector<int> face_, PlateMesh *mesh) {
	this->face_ = face_;
	this->mesh = mesh; 
	recompute_triangulation();
} 

void Plate::draw() {
	vector<P3D> face = get_face_as_vecP3D();
	vector<V3D> uvn = get_uvn();

	// NOTE (!) This only works for convex polygons.
	/*set_color(n_color);
	glBegin(GL_POLYGON); {
		glvecP3D(face);
	} glEnd();*/
	set_color(get_n_color());
	glBegin(GL_TRIANGLES); {
		for (auto &tri : tri_shew) {
			glvecP3D(get_shew_tri_as_vecP3D(tri));
		}
	} glEnd();
 
	glPushAttrib(GL_ALL_ATTRIB_BITS); {
		// --
		glLineWidth((GLfloat) .2);
		set_color(WHITE);
		glBalledCylinderLoop(get_face_as_vecP3D());

		/*
		glLineWidth(2.);
		set_color(get_n_color());
		for (auto &tri : tri_shew) { 
			glBalledCylinderLoop(get_shew_tri_as_vecP3D(tri)); 
		}
		*/
	} glPopAttrib();

	if (mesh->DRAW_LOCAL_COORDINATES) {
		P3D p = vecP3D_sum(face) / double(face.size());
		auto arrow3d_wrapper = [p](V3D n, P3D color) {
			set_color(color);
			draw_arrow3d(p, .025, .125, n);
		};
		arrow3d_wrapper(get_n(), get_n_color());
		arrow3d_wrapper(uvn[0], color_tint(get_n_color()));
		arrow3d_wrapper(uvn[1], color_shade(get_n_color()));
	}

}

P3D Plate::get_n_color() {
	P3D xyz = P3D((V3D(1,1,1) + get_n()).normalized()); // TODO: [g/s]et_col_n
	return P3D(xyz[1], xyz[2], xyz[0]);
} 

vector<P3D> Plate::get_face_as_vecP3D() {
	vector<P3D> tmp;
	for (auto &i : face_) { tmp.push_back(mesh->points[i]); }
	return tmp;
}

vector<P3D> Plate::get_shew_tri_as_vecP3D(const vector<int> &tri) {
	vector<P3D> tmp;
	for (auto &i : tri) { tmp.push_back(v_shew[i]); }
	return tmp;
}

V3D Plate::get_n() {
	vector<P3D> face = get_face_as_vecP3D();
	V3D u = V3D(face[1], face[2]);
	V3D v = V3D(face[1], face[0]);
	return (u.cross(v)).normalized(); // FORNOW
}

vector<V3D> Plate::get_uvn() {
	V3D n = get_n();
	vector<V3D> uvn = { V3D(), V3D(), n };
	n.getOrthogonalVectors(uvn[0], uvn[1]);
	return uvn;
}

P3D Plate::get_t() {
	vector<P3D> face = get_face_as_vecP3D();
	return accumulate(face.begin(), face.end(), P3D()) / double(face.size());
}

shared_ptr<Plane> Plate::get_Pi() {
	return shared_ptr<Plane>(new Plane(get_t(), get_n()));
}

Frame Plate::get_frame() {
	return Frame(get_uvn(), get_t());
}

P3D Plate::hom_mm_extract(const Matrix4x4 &M, const P3D &s) {
	Vector4d s_hom; s_hom.setZero();
	s_hom[0] = s[0];
	s_hom[1] = s[1];
	s_hom[2] = s[2];
	s_hom[3] = 1.;
	Vector4d res = M *s_hom;
	return P3D(res[0]/res[3], res[1]/res[3], res[2]/res[3]);
}

P3D Plate::O2uvnt(const P3D &s) {
	return get_frame().world2local(s);
}

P3D Plate::uvnt2O(const P3D &s) {
	return get_frame().local2world(s);
}

bool Plate::triangle_contains(const vector<P3D> &abc, const P3D &s) {
	if (abc.size() != 3) { error("InputError"); }
	// --
	vector<P3D> abc_prime = { O2uvnt(abc[0]),
	                          O2uvnt(abc[1]),
	                          O2uvnt(abc[2]) };
	P3D s_prime =             O2uvnt(     s);

	auto IS_SMALL = [](const double &a) { return abs(a) < 10e-5; };
	if (!IS_SMALL(abc_prime[0].z())) { cout << abc_prime[0].transpose(); error("SeeMe:a"); return false; }
	if (!IS_SMALL(abc_prime[1].z())) { cout << abc_prime[1].transpose(); error("SeeMe:b"); return false; }
	if (!IS_SMALL(abc_prime[2].z())) { cout << abc_prime[2].transpose(); error("SeeMe:c"); return false; }
	if (!IS_SMALL(     s_prime.z())) { /*cout <<      s_prime.transpose(); error("SeeMe:s");*/ return false; }

	auto lambda = get_bary_lambda_2d(abc_prime, s_prime);
	return (lambda[0] >= 0. && lambda[1] >= 0. && lambda[2] >= 0); 
}

bool Plate::contains(const P3D &s) { 
	for (auto &tri : tri_shew) {
		if (triangle_contains(get_shew_tri_as_vecP3D(tri), s)) {
			return true;
		}
	}
	return false;
}

void Plate::recompute_triangulation() {
	vector<V3D> uvn = get_uvn();
	V3D t = get_t(); // Any point on triangle (in world space) 
	// -- 
	vector<P3D> face_O = get_face_as_vecP3D(); 
	vector<P3D> face_uvnt; for (auto &s : face_O) { face_uvnt.push_back(O2uvnt(s)); }

	// --

	vector<P3D> &face = face_uvnt;

	triangulateio in, out;

	in.numberofpoints = face.size();
	in.pointlist = (REAL *)malloc(2 * face.size() * sizeof(REAL));
	{
		int k = 0;
		for (auto &p : face) {
			in.pointlist[k++] = p[0];
			in.pointlist[k++] = p[1];
		}
	}

	in.numberofpointattributes = 0;
	in.pointattributelist = (double *)NULL;
	in.pointmarkerlist = (int *)NULL;

	in.numberofsegments = face.size();
	in.segmentlist = (int *)malloc(2 * face.size() * sizeof(int));
	{
		int k = 0;
		for (size_t i = 0; i < face.size(); ++i) {
			in.segmentlist[k++] = i;
			in.segmentlist[k++] = (i + 1) % face.size();
		}
	}

	in.segmentmarkerlist = (int *)NULL;

	in.numberofholes = 0;
	in.holelist = (REAL *)NULL;

	in.numberofregions = 0;
	in.regionlist = (double *)NULL;

	out.pointlist = (REAL *)NULL;
	out.trianglelist = (int *)NULL;
	out.segmentlist = (int *)NULL;
	out.segmentmarkerlist = (int *)NULL;
	out.edgelist = (int *)NULL;

	char args[128];
	sprintf(args, "pzAeBQa");// q32.5");
	triangulate(args, &in, &out, (struct triangulateio *)NULL);

	// -- store
	v_shew.clear();
	tri_shew.clear();

	{
		for (int i = 0; i < out.numberofpoints; ++i) {
			double x = out.pointlist[2 * i];
			double y = out.pointlist[2 * i + 1];
			P3D s_prime = P3D(x, y);
			v_shew.push_back(uvnt2O(s_prime)); // (*)
		}
	}

	{
		for (int i = 0; i < out.numberoftriangles; i++) {
			vector<int> triangle;
			for (int j = 0; j < 3; ++j) {
				int k = out.trianglelist[3 * i + j];
				triangle.push_back(k);
			}
			tri_shew.push_back(triangle);
		}
	}

	// -- clean
	std::free(in.pointlist);
	std::free(in.segmentlist);
	std::free(in.holelist);
	std::free(out.pointlist);
	std::free(out.trianglelist);
	std::free(out.segmentlist);
	std::free(out.edgelist); 
}