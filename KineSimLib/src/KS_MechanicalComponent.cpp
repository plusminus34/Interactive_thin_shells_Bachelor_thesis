#include "KineSimLib/KS_MechanicalComponent.h"
#include "KineSimLib/KS_LoaderUtils.h"
#include <MathLib/Triangle.h>

int KS_MechanicalComponent::m_stateSize = 6;

KS_MechanicalComponent::KS_MechanicalComponent(const char* name){
	setName(name);
	position = Point3d(0,0,0);

	//we'll assume the main rotation axis (around which the phase/alpha angle operates) is fixed
	n_alpha = Vector3d(0,0,1);
	//NOTE: the two axis below should be a function of the global orientation that is expected for the component. The good thing is that
	//these should be swapable at run-time, as the angle beta goes close to pi/2 or -pi/2. For now we'll assume they're constant
	n_beta = Vector3d(0,1,0);
	n_gamma = Vector3d(1,0,0);
	setAngles(0, 0, 0);

	selected = false;
	meshColor = ThreeTuple(0.5, 0.5, 0.8);
	layerNumber = 0;
}

KS_MechanicalComponent::~KS_MechanicalComponent(void){
	for (uint i=0;i<meshes.size();i++)
		delete meshes[i];
}

//w represents the world coordinates for the point (or vector) x, which is expressed in the local coordinate frame of the component. 
Point3d KS_MechanicalComponent::get_x(const Point3d& w) const{
	return (R_gamma*R_beta*R_alpha).getInverse()*Vector3d(position,w);
}

//w represents the world coordinates for the point (or vector) x, which is expressed in the local coordinate frame of the component. 
Vector3d KS_MechanicalComponent::get_x(const Vector3d& w) const{
	return (R_gamma*R_beta*R_alpha).getInverse()*w;
}

//w represents the world coordinates for the point (or vector) x, which is expressed in the local coordinate frame of the component. The function works both for local points or vectors
Point3d KS_MechanicalComponent::get_w(const Point3d& x) const {
	//w = (R*x) + p where R is the overall rotation, and p is the global position of the component
	return R_gamma*(R_beta*(R_alpha*x)) + position;
}

//w represents the world coordinates for the point (or vector) x, which is expressed in the local coordinate frame of the component. The function works both for local points or vectors
Vector3d KS_MechanicalComponent::get_w(const Vector3d& x) const {
	//w = (R*x) where R is the overall rotation
	return R_gamma*(R_beta*(R_alpha*x));
}

//return the jacobian that relates the change in world coordinates w with the change in state s=(alpha, p.x, p.y, p.z)
void KS_MechanicalComponent::get_dw_ds(const Point3d& x, Matrix& dw_ds){
	//given that dR(alpha)/dalpha = n x R (where n is the axis of rotation), it is easy to compute derivatives of w wrt s...
	assert(dw_ds.getRowCount() == 3);
	assert(dw_ds.getColCount() == getStateSize());

	Vector3d dw_dalpha = R_gamma*(R_beta*(n_alpha.cross(R_alpha*x)));
	Vector3d dw_dbeta  = R_gamma*( n_beta.cross(R_beta*(R_alpha*x)));
	Vector3d dw_dgamma = n_gamma.cross(R_gamma*(R_beta*(R_alpha*x)));

	dw_ds.setToZeros();
	for (int i=0;i<3;i++){
		dw_ds(i,0) = dw_dgamma[i];
		dw_ds(i,1) =  dw_dbeta[i];
		dw_ds(i,2) = dw_dalpha[i];
		dw_ds(i,3+i) = 1.0;
	}
}

//return the jacobian that relates the change in world coordinates w with the change in state s=(alpha, p.x, p.y, p.z)
void KS_MechanicalComponent::get_dw_ds(const Vector3d& x, Matrix& dw_ds){
	//given that dR(alpha)/dalpha = n x R (where n is the axis of rotation), it is easy to compute derivatives of w wrt s...
	assert(dw_ds.getRowCount() == 3);
	assert(dw_ds.getColCount() == getStateSize());

	Vector3d dw_dalpha = R_gamma*(R_beta*(n_alpha.cross(R_alpha*x)));
	Vector3d dw_dbeta  = R_gamma*( n_beta.cross(R_beta*(R_alpha*x)));
	Vector3d dw_dgamma = n_gamma.cross(R_gamma*(R_beta*(R_alpha*x)));

	dw_ds.setToZeros();
	for (int i=0;i<3;i++){
		dw_ds(i,0) = dw_dgamma[i];
		dw_ds(i,1) =  dw_dbeta[i];
		dw_ds(i,2) = dw_dalpha[i];
	}
}


//return the matrix that relates the change in change of w with the change in rotation angle alpha
void KS_MechanicalComponent::get_ddw_dads(const Point3d& x, Matrix& ddw_das){
	Vector3d ddw_daa = R_gamma*(R_beta*(n_alpha.cross(n_alpha.cross(R_alpha*x))));
	Vector3d ddw_dab = R_gamma*(n_beta.cross(R_beta*(n_alpha.cross(R_alpha*x))));
	Vector3d ddw_dag = n_gamma.cross(R_gamma*(R_beta*(n_alpha.cross(R_alpha*x))));

	ddw_das.setToZeros();
	for (int i=0;i<3;i++){
		ddw_das(i,0) = ddw_dag[i];
		ddw_das(i,1) =  ddw_dab[i];
		ddw_das(i,2) = ddw_daa[i];
	}
}


//return the matrix that relates the change in change of w with the change in rotation angle alpha
void KS_MechanicalComponent::get_ddw_dads(const Vector3d& x, Matrix& ddw_das){
	Vector3d ddw_daa = R_gamma*(R_beta*(n_alpha.cross(n_alpha.cross(R_alpha*x))));
	Vector3d ddw_dab = R_gamma*(n_beta.cross(R_beta*(n_alpha.cross(R_alpha*x))));
	Vector3d ddw_dag = n_gamma.cross(R_gamma*(R_beta*(n_alpha.cross(R_alpha*x))));

	ddw_das.setToZeros();
	for (int i=0;i<3;i++){
		ddw_das(i,0) = ddw_dag[i];
		ddw_das(i,1) =  ddw_dab[i];
		ddw_das(i,2) = ddw_daa[i];
	}
}

//return the matrix that relates the change in change of w with the change in rotation angle beta
void KS_MechanicalComponent::get_ddw_dbds(const Point3d& x, Matrix& ddw_dbs){
	Vector3d ddw_dba = R_gamma*(n_beta.cross(R_beta*(n_alpha.cross(R_alpha*x))));
	Vector3d ddw_dbb = R_gamma*(n_beta.cross(n_beta.cross(R_beta*(R_alpha*x))));
	Vector3d ddw_dbg = n_gamma.cross(R_gamma*(n_beta.cross(R_beta*(R_alpha*x))));

	ddw_dbs.setToZeros();
	for (int i=0;i<3;i++){
		ddw_dbs(i,0) = ddw_dbg[i];
		ddw_dbs(i,1) = ddw_dbb[i];
		ddw_dbs(i,2) = ddw_dba[i];
	}
}

//return the matrix that relates the change in change of w with the change in rotation angle beta
void KS_MechanicalComponent::get_ddw_dbds(const Vector3d& x, Matrix& ddw_dbs){
	Vector3d ddw_dba = R_gamma*(n_beta.cross(R_beta*(n_alpha.cross(R_alpha*x))));
	Vector3d ddw_dbb = R_gamma*(n_beta.cross(n_beta.cross(R_beta*(R_alpha*x))));
	Vector3d ddw_dbg = n_gamma.cross(R_gamma*(n_beta.cross(R_beta*(R_alpha*x))));

	ddw_dbs.setToZeros();
	for (int i=0;i<3;i++){
		ddw_dbs(i,0) = ddw_dbg[i];
		ddw_dbs(i,1) = ddw_dbb[i];
		ddw_dbs(i,2) = ddw_dba[i];
	}
}


//return the matrix that relates the change in change of w with the change in rotation angle gamma
void KS_MechanicalComponent::get_ddw_dgds(const Point3d& x, Matrix& ddw_dgs){
	Vector3d ddw_dga = n_gamma.cross(R_gamma*(R_beta*(n_alpha.cross(R_alpha*x))));
	Vector3d ddw_dgb = n_gamma.cross(R_gamma*( n_beta.cross(R_beta*(R_alpha*x))));
	Vector3d ddw_dgg = n_gamma.cross(n_gamma.cross(R_gamma*(R_beta*(R_alpha*x))));

	ddw_dgs.setToZeros();
	for (int i=0;i<3;i++){
		ddw_dgs(i,0) = ddw_dgg[i];
		ddw_dgs(i,1) = ddw_dgb[i];
		ddw_dgs(i,2) = ddw_dga[i];
	}
}


//return the matrix that relates the change in change of w with the change in rotation angle gamma
void KS_MechanicalComponent::get_ddw_dgds(const Vector3d& x, Matrix& ddw_dgs){
	Vector3d ddw_dga = n_gamma.cross(R_gamma*(R_beta*(n_alpha.cross(R_alpha*x))));
	Vector3d ddw_dgb = n_gamma.cross(R_gamma*( n_beta.cross(R_beta*(R_alpha*x))));
	Vector3d ddw_dgg = n_gamma.cross(n_gamma.cross(R_gamma*(R_beta*(R_alpha*x))));

	ddw_dgs.setToZeros();
	for (int i=0;i<3;i++){
		ddw_dgs(i,0) = ddw_dgg[i];
		ddw_dgs(i,1) = ddw_dgb[i];
		ddw_dgs(i,2) = ddw_dga[i];
	}
}


void KS_MechanicalComponent::setWorldCenterPosition(const Point3d& pos){
	position = pos;
}

Point3d KS_MechanicalComponent::getWorldCenterPosition() const{
	return position;
}

void KS_MechanicalComponent::updateTracerParticles(){
	for (uint i=0;i<tracerParticles.size();i++){
		tracerParticles[i].addTrajectoryPoint();
	}
}

void KS_MechanicalComponent::clearTracerParticles(){
	for (uint i=0;i<tracerParticles.size();i++){
		tracerParticles[i].trajectory.clear();
	}
}

void KS_MechanicalComponent::addTracerParticlesToList(DynamicArray<Point3d>& tracerParticleList){
	for (uint i=0;i<tracerParticles.size();i++)
		tracerParticleList.push_back(get_w(tracerParticles[i].pLocal));
}

Point3d KS_MechanicalComponent::getTracerParticlePosition(int i){
	if ((uint)i<0 || (uint)i>=tracerParticles.size())
		return get_w(Point3d(0,0,0));
	return get_w(tracerParticles[i].pLocal);
}

Point3d KS_MechanicalComponent::getTracerParticleLocalPosition(int i){
	if ((uint)i<0 || (uint)i>=tracerParticles.size())
		return Point3d(0,0,0);
	return tracerParticles[i].pLocal;
}

void KS_MechanicalComponent::setPhase(double a){
	alpha = a;
	R_alpha = Quaternion::getRotationQuaternion(alpha, n_alpha);
}

void KS_MechanicalComponent::setAngles(double val_gamma, double val_beta, double val_alpha) { 
	alpha = val_alpha;
	beta = val_beta;
	gamma = val_gamma;

	R_alpha = Quaternion::getRotationQuaternion(alpha, n_alpha);
	R_beta = Quaternion::getRotationQuaternion(beta, n_beta);
	R_gamma = Quaternion::getRotationQuaternion(gamma, n_gamma);

	//check for singularities. If it's a X-Y-X type of euler angle configuration, then the second angle needs to stay away from 0 and pi.
	//if it's Z-Y-X, then it needs to stay away from -pi/2 and pi/2
	if (n_alpha.dotProductWith(n_gamma) > 0.99 || n_alpha.dotProductWith(n_gamma) < -0.99){
		if (beta < 0.2 || beta > PI - 0.2){
//			logPrint("Component orientation getting close to singularity! Should consider changing Euler angle representation\n");
			assert(false);
		}
	}else{
		if (beta < -PI / 2 + 0.2 || beta > PI/2 - 0.2){
//			logPrint("Component orientation getting close to singularity! Should consider changing Euler angle representation\n");
//			assert(false);
		}
	}

}

void KS_MechanicalComponent::writeBaseComponentToFile(FILE* f){
	char* str;

	str = getKSString(KS_NAME);
	fprintf(f, "\t%s %s\n", str, m_name);
	str = getKSString(KS_COLOR);
	fprintf(f, "\t%s %lf %lf %lf\n", str, meshColor[0], meshColor[1], meshColor[2]);
	str = getKSString(KS_POSITION_IN_WORLD);
	Point3d tmpP = getWorldCenterPosition();
	fprintf(f, "\t%s %lf %lf %lf\n", str, tmpP[0], tmpP[1], tmpP[2]);
	str = getKSString(KS_ANGLE);
	fprintf(f, "\t%s %lf\n", str, alpha);
	str = getKSString(KS_BETA);
	fprintf(f, "\t%s %lf\n", str, beta);
	str = getKSString(KS_GAMMA);
	fprintf(f, "\t%s %lf\n", str, gamma);
	str = getKSString(KS_BETA_AXIS);
	fprintf(f, "\t%s %lf %lf %lf\n", str, n_beta[0], n_beta[1], n_beta[2]);
	str = getKSString(KS_GAMMA_AXIS);
	fprintf(f, "\t%s %lf %lf %lf\n", str, n_gamma[0], n_gamma[1], n_gamma[2]);
	str = getKSString(KS_LAYER_NUMBER);
	fprintf(f, "\t%s %d \n", str, layerNumber);

	for (uint i=0; i<tracerParticles.size();i++){
		str = getKSString(KS_TRACER_PARTICLE);
		fprintf(f, "\t%s %lf %lf %lf\n", str, tracerParticles[i].pLocal[0], tracerParticles[i].pLocal[1], tracerParticles[i].pLocal[2]);
	}

}


//returns true if the input line was processed, false otherwise
bool KS_MechanicalComponent::processInputLine(char* line){
	int lineType = getKSLineType(line);
	switch (lineType) {
		case KS_NAME:
			strcpy(m_name, trim(line));
			return true;
			break;
		case KS_POSITION_IN_WORLD:
			if (sscanf(line, "%lf %lf %lf", &position.x, &position.y, &position.z) != 3) assert(false);
			return true;
			break;
		case KS_ANGLE:
			if (sscanf(line, "%lf", &alpha) != 1) assert(false);
			setAngles(gamma, beta, alpha);
			return true;
			break;
		case KS_COLOR:
			if (sscanf(line, "%lf %lf %lf", &meshColor.x, &meshColor.y, &meshColor.z) != 3) assert(false);
			return true;
			break;
		case KS_TRACER_PARTICLE:{
				Point3d tmpP;
				if (sscanf(line, "%lf %lf %lf", &tmpP.x, &tmpP.y, &tmpP.z) != 3) assert(false);
				tracerParticles.push_back(TracerParticle(this, tmpP));
				return true;
			}
			break;
		case KS_BETA_AXIS:
			if (sscanf(line, "%lf %lf %lf", &n_beta.x, &n_beta.y, &n_beta.z) != 3) assert(false);
			n_beta.normalize();
			setAngles(gamma, beta, alpha);
			return true;
			break;
		case KS_GAMMA_AXIS:
			if (sscanf(line, "%lf %lf %lf", &n_gamma.x, &n_gamma.y, &n_gamma.z) != 3) assert(false);
			n_gamma.normalize();
			setAngles(gamma, beta, alpha);
			return true;
			break;
		case KS_BETA:
			if (sscanf(line, "%lf", &beta) != 1) assert(false);
			setAngles(gamma, beta, alpha);
			return true;
			break;
		case KS_GAMMA:
			if (sscanf(line, "%lf", &gamma) != 1) assert(false);
			setAngles(gamma, beta, alpha);
			return true;
			break;
		case KS_LAYER_NUMBER:
			if (sscanf(line, "%d", &layerNumber) != 1) assert(false);
			return true;
			break;
	}
	return false;
}

void KS_MechanicalComponent::addCylinderMesh(int nrVerts, double radius, double length, Point3d localCoords, Vector3d v, bool setMeshColor){
  //we'll start out by getting a vector that is perpendicular to the given vector.

	double rotAngle = safeACOS(v.dotProductWith(Vector3d(0,0,1)));
	Vector3d rotAxis = v.crossProductWith(Vector3d(0,0,1)).toUnit();

	if (IS_ZERO(rotAngle)) rotAxis = Vector3d(1,0,0);

	GLMesh* tmpMesh = new GLMesh();

	for (int i=0;i<nrVerts;i++){
		double angle = 2 * PI / nrVerts * i;

		tmpMesh->addVertex(localCoords+Vector3d(cos(angle) * radius, sin(angle) * radius, length/2.0).rotate(rotAngle, rotAxis));
		tmpMesh->addVertex(localCoords+Vector3d(cos(angle) * radius, sin(angle) * radius, -length/2.0).rotate(rotAngle, rotAxis));
	}

	for (int i=0; i<nrVerts;i++){
		int nextIndex = i+1;
		if (nextIndex == nrVerts) nextIndex = 0;
		GLIndexedPoly poly;
		poly.addVertexIndex(2*i);poly.addVertexIndex(2*i+1);poly.addVertexIndex(2*nextIndex+1);poly.addVertexIndex(2*nextIndex);
		tmpMesh->addPoly(poly);
	}

	int middleVertexIndex = (int)tmpMesh->getVertexCount();
	tmpMesh->addVertex(localCoords + Vector3d(0, 0, length/2.0).rotate(rotAngle, rotAxis));
	tmpMesh->addVertex(localCoords + Vector3d(0, 0, -length/2.0).rotate(rotAngle, rotAxis));

	for (int i=0; i<nrVerts; i++){
		int nextIndex = i+1;
		if (nextIndex == nrVerts) nextIndex = 0;
		GLIndexedPoly poly;
		poly.addVertexIndex(2*i);poly.addVertexIndex(2*nextIndex);poly.addVertexIndex(middleVertexIndex);
		tmpMesh->addPoly(poly);
		poly.clear();
		poly.addVertexIndex(2*i+1);poly.addVertexIndex(middleVertexIndex+1);poly.addVertexIndex(2*nextIndex+1);
		tmpMesh->addPoly(poly);
	}

	if (setMeshColor)
		tmpMesh->setColour(meshColor[0], meshColor[1], meshColor[2], 1);
	tmpMesh->computeNormals();
	meshes.push_back(tmpMesh);
}

void KS_MechanicalComponent::loadTriangleMeshFromFile(char* fName){
	GLMesh* tmpMesh = OBJReader::loadOBJFile(fName);
	tmpMesh->computeNormals();
	tmpMesh->setColour(meshColor[0], meshColor[1], meshColor[2], 1);
	meshes.push_back(tmpMesh);
}

void TracerParticle::addTrajectoryPoint(){
	trajectory.push_back(mc->get_w(pLocal));
}

/**
	This method draws the meshes of the component.
*/
void KS_MechanicalComponent::draw(){
  #ifdef DISPLAY
	glPushMatrix();

	TransformationMatrix toWorld;
	Quaternion qToWorld = R_gamma*R_beta*R_alpha;
	qToWorld.getRotationMatrix(&toWorld);
	toWorld.setTranslation(position + Vector3d(0, 0, 1) * layerNumber * LAYER_THICKNESS);

	double values[16];
	toWorld.getOGLValues(values);
	glMultMatrixd(values);

	for (uint i=0;i<meshes.size();i++)
		meshes[i]->drawMesh(!selected);

	glPopMatrix();
#endif	
}


/**
	This method draws the tracer particles
*/
void KS_MechanicalComponent::drawTracerParticles(){
  #ifdef DISPLAY
	glLineWidth(2.0);
	for (uint i=0;i<tracerParticles.size();i++){
		glColor3f(0.0f,1.0f,0);
		glBegin(GL_LINE_STRIP);
		for (uint j=0;j<tracerParticles[i].trajectory.size();j++)
			glVertex3d(tracerParticles[i].trajectory[j].x, tracerParticles[i].trajectory[j].y, tracerParticles[i].trajectory[j].z);
		glEnd();
	}
#endif	
}


//returns true if the ray intersects the object, false otherwise...
bool KS_MechanicalComponent::isIntersectedByRay(const Ray& r, Point3d& res){
	//TODO: we might want to reintroduce the AABB as a first test, perhaps even a bounding/encapsulating sphere
	Ray localCoordRay(get_x(r.origin), get_x(r.direction));
	for (uint i=0;i<meshes.size();i++){
		GLMesh* mesh = meshes[i];
		for (uint j=0; j<mesh->triangles.size(); j++){
			//go through all the triangles and the quads and do collision checks....
			GLIndexedTriangle& git = mesh->triangles[j];
			Triangle triangle(mesh->getVertex(git.indexes[0]), mesh->getVertex(git.indexes[1]), mesh->getVertex(git.indexes[2]));
			if (triangle.isIntersectedByRay(localCoordRay, res)){
				res = get_w(res);
				return true;
			}
		}
	}

	return false;
}

uint KS_MechanicalComponent::renderToObjFile(FILE* fp, uint vertexIdxOffset, double scale)
{
	TransformationMatrix mat;
	Quaternion quat = R_gamma*R_beta*R_alpha;

	quat.getRotationMatrix(&mat);
	mat.setTranslation(this->position);
	mat.setScale(scale);

	uint retVal = 0;
	for (uint i=0;i<meshes.size();i++)
		retVal += meshes[i]->renderToObjFile( fp, vertexIdxOffset + retVal, mat);
	return retVal;
}

AABoundingBox KS_MechanicalComponent::computeAABB(){
	Point3d AABB_blf(DBL_MAX, DBL_MAX, DBL_MAX);
	Point3d AABB_trb(-DBL_MAX, -DBL_MAX, -DBL_MAX);

	for (uint i=0; i<meshes.size();i++){
		for (int j=0; j<meshes[i]->getVertexCount(); j++){
			Point3d p = get_w(meshes[i]->getVertex(j));
			if (p.x < AABB_blf.x) AABB_blf.x = p.x;
			if (p.y < AABB_blf.y) AABB_blf.y = p.y;
			if (p.z < AABB_blf.z) AABB_blf.z = p.z;

			if (p.x > AABB_trb.x) AABB_trb.x = p.x;
			if (p.y > AABB_trb.y) AABB_trb.y = p.y;
			if (p.z > AABB_trb.z) AABB_trb.z = p.z;
		}
	}

	return AABoundingBox(AABB_blf, AABB_trb);
}

