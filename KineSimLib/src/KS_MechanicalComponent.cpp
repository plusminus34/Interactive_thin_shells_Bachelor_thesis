#include "KineSimLib/KS_MechanicalComponent.h"
#include "KineSimLib/KS_LoaderUtils.h"
#include <MathLib/Quaternion.h>
#include <MathLib/Transformation.h>
#include <GUILib/GLUtils.h>


int KS_MechanicalComponent::m_stateSize = 6;

KS_MechanicalComponent::KS_MechanicalComponent(const char* name){
	//Logger::print("Constructor mechanical component calld \n");
	setName(name);
	position = P3D(0,0,0);

	//we'll assume the main rotation axis (around which the phase/alpha angle operates) is fixed
	n_alpha = V3D(0,0,1);
	//NOTE: the two axis below should be a function of the global orientation that is expected for the component. The good thing is that
	//these should be swapable at run-time, as the angle beta goes close to pi/2 or -pi/2. For now we'll assume they're constant
	n_beta = V3D(0,1,0);
	n_gamma = V3D(1,0,0);
	setAngles(0, 0, 0);

	selected = false;
	layerNumber = 0;
	points_list.clear();
	addCubeAroundApoint(position, 0.05);
}

KS_MechanicalComponent::~KS_MechanicalComponent(void){
	for (uint i=0;i<meshes.size();i++)
		delete meshes[i];
}

//w represents the world coordinates for the point (or vector) x, which is expressed in the local coordinate frame of the component. 
P3D KS_MechanicalComponent::get_x(const P3D& w) const{
	return (R_gamma*R_beta*R_alpha).getInverse()*V3D(position,w);
}

//w represents the world coordinates for the point (or vector) x, which is expressed in the local coordinate frame of the component. 
V3D KS_MechanicalComponent::get_x(const V3D& w) const{
	return (R_gamma*R_beta*R_alpha).getInverse()*w;
}

//w represents the world coordinates for the point (or vector) x, which is expressed in the local coordinate frame of the component. The function works both for local points or vectors
P3D KS_MechanicalComponent::get_w(const P3D& x) const {
	//w = (R*x) + p where R is the overall rotation, and p is the global position of the component
	return R_gamma*(R_beta*(R_alpha*x)) + position;
}

//w represents the world coordinates for the point (or vector) x, which is expressed in the local coordinate frame of the component. The function works both for local points or vectors
V3D KS_MechanicalComponent::get_w(const V3D& x) const {
	//w = (R*x) where R is the overall rotation
	return R_gamma*(R_beta*(R_alpha*x));
}

//return the jacobian that relates the change in world coordinates w with the change in state s=(alpha, p.x, p.y, p.z)
void KS_MechanicalComponent::get_dw_ds(const P3D& x, MatrixNxM& dw_ds){
	//given that dR(alpha)/dalpha = n x R (where n is the axis of rotation), it is easy to compute derivatives of w wrt s...
	assert(dw_ds.rows() == 3);
	assert(dw_ds.cols() == getStateSize());

	V3D dw_dalpha = R_gamma*(R_beta*(n_alpha.cross(R_alpha*x)));
	V3D dw_dbeta  = R_gamma*( n_beta.cross(R_beta*(R_alpha*x)));
	V3D dw_dgamma = n_gamma.cross(R_gamma*(R_beta*(R_alpha*x)));

	dw_ds.setZero();
	for (int i=0;i<3;i++){
		dw_ds(i,0) = dw_dgamma[i];
		dw_ds(i,1) =  dw_dbeta[i];
		dw_ds(i,2) = dw_dalpha[i];
		dw_ds(i,3+i) = 1.0;
	}
}

//return the jacobian that relates the change in world coordinates w with the change in state s=(alpha, p.x, p.y, p.z)
void KS_MechanicalComponent::get_dw_ds(const V3D& x, MatrixNxM& dw_ds){
	//given that dR(alpha)/dalpha = n x R (where n is the axis of rotation), it is easy to compute derivatives of w wrt s...
	assert(dw_ds.rows() == 3);
	assert(dw_ds.cols() == getStateSize());

	V3D dw_dalpha = R_gamma*(R_beta*(n_alpha.cross(R_alpha*x)));
	V3D dw_dbeta  = R_gamma*( n_beta.cross(R_beta*(R_alpha*x)));
	V3D dw_dgamma = n_gamma.cross(R_gamma*(R_beta*(R_alpha*x)));

	dw_ds.setZero();
	for (int i=0;i<3;i++){
		dw_ds(i,0) = dw_dgamma[i];
		dw_ds(i,1) =  dw_dbeta[i];
		dw_ds(i,2) = dw_dalpha[i];
	}
}


//return the matrix that relates the change in change of w with the change in rotation angle alpha
void KS_MechanicalComponent::get_ddw_dads(const P3D& x, MatrixNxM& ddw_das){
	V3D ddw_daa = R_gamma*(R_beta*(n_alpha.cross(n_alpha.cross(R_alpha*x))));
	V3D ddw_dab = R_gamma*(n_beta.cross(R_beta*(n_alpha.cross(R_alpha*x))));
	V3D ddw_dag = n_gamma.cross(R_gamma*(R_beta*(n_alpha.cross(R_alpha*x))));

	ddw_das.setZero();
	for (int i=0;i<3;i++){
		ddw_das(i,0) = ddw_dag[i];
		ddw_das(i,1) =  ddw_dab[i];
		ddw_das(i,2) = ddw_daa[i];
	}
}


//return the matrix that relates the change in change of w with the change in rotation angle alpha
void KS_MechanicalComponent::get_ddw_dads(const V3D& x, MatrixNxM& ddw_das){
	V3D ddw_daa = R_gamma*(R_beta*(n_alpha.cross(n_alpha.cross(R_alpha*x))));
	V3D ddw_dab = R_gamma*(n_beta.cross(R_beta*(n_alpha.cross(R_alpha*x))));
	V3D ddw_dag = n_gamma.cross(R_gamma*(R_beta*(n_alpha.cross(R_alpha*x))));

	ddw_das.setZero();
	for (int i=0;i<3;i++){
		ddw_das(i,0) = ddw_dag[i];
		ddw_das(i,1) =  ddw_dab[i];
		ddw_das(i,2) = ddw_daa[i];
	}
}

//return the matrix that relates the change in change of w with the change in rotation angle beta
void KS_MechanicalComponent::get_ddw_dbds(const P3D& x, MatrixNxM& ddw_dbs){
	V3D ddw_dba = R_gamma*(n_beta.cross(R_beta*(n_alpha.cross(R_alpha*x))));
	V3D ddw_dbb = R_gamma*(n_beta.cross(n_beta.cross(R_beta*(R_alpha*x))));
	V3D ddw_dbg = n_gamma.cross(R_gamma*(n_beta.cross(R_beta*(R_alpha*x))));

	ddw_dbs.setZero();
	for (int i=0;i<3;i++){
		ddw_dbs(i,0) = ddw_dbg[i];
		ddw_dbs(i,1) = ddw_dbb[i];
		ddw_dbs(i,2) = ddw_dba[i];
	}
}

//return the matrix that relates the change in change of w with the change in rotation angle beta
void KS_MechanicalComponent::get_ddw_dbds(const V3D& x, MatrixNxM& ddw_dbs){
	V3D ddw_dba = R_gamma*(n_beta.cross(R_beta*(n_alpha.cross(R_alpha*x))));
	V3D ddw_dbb = R_gamma*(n_beta.cross(n_beta.cross(R_beta*(R_alpha*x))));
	V3D ddw_dbg = n_gamma.cross(R_gamma*(n_beta.cross(R_beta*(R_alpha*x))));

	ddw_dbs.setZero();
	for (int i=0;i<3;i++){
		ddw_dbs(i,0) = ddw_dbg[i];
		ddw_dbs(i,1) = ddw_dbb[i];
		ddw_dbs(i,2) = ddw_dba[i];
	}
}


//return the matrix that relates the change in change of w with the change in rotation angle gamma
void KS_MechanicalComponent::get_ddw_dgds(const P3D& x, MatrixNxM& ddw_dgs){
	V3D ddw_dga = n_gamma.cross(R_gamma*(R_beta*(n_alpha.cross(R_alpha*x))));
	V3D ddw_dgb = n_gamma.cross(R_gamma*( n_beta.cross(R_beta*(R_alpha*x))));
	V3D ddw_dgg = n_gamma.cross(n_gamma.cross(R_gamma*(R_beta*(R_alpha*x))));

	ddw_dgs.setZero();
	for (int i=0;i<3;i++){
		ddw_dgs(i,0) = ddw_dgg[i];
		ddw_dgs(i,1) = ddw_dgb[i];
		ddw_dgs(i,2) = ddw_dga[i];
	}
}


//return the matrix that relates the change in change of w with the change in rotation angle gamma
void KS_MechanicalComponent::get_ddw_dgds(const V3D& x, MatrixNxM& ddw_dgs){
	V3D ddw_dga = n_gamma.cross(R_gamma*(R_beta*(n_alpha.cross(R_alpha*x))));
	V3D ddw_dgb = n_gamma.cross(R_gamma*( n_beta.cross(R_beta*(R_alpha*x))));
	V3D ddw_dgg = n_gamma.cross(n_gamma.cross(R_gamma*(R_beta*(R_alpha*x))));

	ddw_dgs.setZero();
	for (int i=0;i<3;i++){
		ddw_dgs(i,0) = ddw_dgg[i];
		ddw_dgs(i,1) = ddw_dgb[i];
		ddw_dgs(i,2) = ddw_dga[i];
	}
}


void KS_MechanicalComponent::setWorldCenterPosition(const P3D& pos){
	position = pos;
}

P3D KS_MechanicalComponent::getWorldCenterPosition() const{
	return position;
}




void KS_MechanicalComponent::setAngles(double val_gamma, double val_beta, double val_alpha) { 
	alpha = val_alpha;
	beta = val_beta;
	gamma = val_gamma;

	R_alpha = getRotationQuaternion(alpha, n_alpha);
	R_beta = getRotationQuaternion(beta, n_beta);
	R_gamma = getRotationQuaternion(gamma, n_gamma);

	//check for singularities. If it's a X-Y-X type of euler angle configuration, then the second angle needs to stay away from 0 and pi.
	//if it's Z-Y-X, then it needs to stay away from -pi/2 and pi/2
	if (n_alpha.dot(n_gamma) > 0.99 || n_alpha.dot(n_gamma) < -0.99){
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
	str = getKSString(KS_POSITION_IN_WORLD);
	P3D tmpP = getWorldCenterPosition();
	fprintf(f, "\t%s %lf %lf %lf\n", str, tmpP[0], tmpP[1], tmpP[2]);
	str = getKSString(KS_ALPHA);
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
			if (sscanf(line, "%lf %lf %lf", &position[0], &position[1], &position[2]) != 3) assert(false);
			return true;
			break;
		case KS_ALPHA:
			if (sscanf(line, "%lf", &alpha) != 1) assert(false);
			setAngles(gamma, beta, alpha);
			return true;
			break;
		case KS_BETA_AXIS:
			if (sscanf(line, "%lf %lf %lf", &n_beta[0], &n_beta[1], &n_beta[2]) != 3) assert(false);
			n_beta.normalize();
			setAngles(gamma, beta, alpha);
			return true;
			break;
		case KS_GAMMA_AXIS:
			if (sscanf(line, "%lf %lf %lf", &n_gamma[0], &n_gamma[1], &n_gamma[2]) != 3) assert(false);
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

void KS_MechanicalComponent::addCylinderMesh(int nrVerts, double radius, double length, P3D localCoords, V3D v, bool setMeshColor){
  //we'll start out by getting a vector that is perpendicular to the given vector.

	double rotAngle = safeACOS(v.dot(V3D(0,0,1)));
	V3D rotAxis = v.cross(V3D(0,0,1)).toUnit();

	if (IS_ZERO(rotAngle)) rotAxis = V3D(1,0,0);

	GLMesh* tmpMesh = new GLMesh();

	for (int i=0;i<nrVerts;i++){
		double angle = 2 * PI / nrVerts * i;

		tmpMesh->addVertex(localCoords+V3D(cos(angle) * radius, sin(angle) * radius, length/2.0).rotate(rotAngle, rotAxis));
		tmpMesh->addVertex(localCoords+V3D(cos(angle) * radius, sin(angle) * radius, -length/2.0).rotate(rotAngle, rotAxis));
	}

	for (int i=0; i<nrVerts;i++){
		int nextIndex = i+1;
		if (nextIndex == nrVerts) nextIndex = 0;
		GLIndexedPoly poly;
		poly.addVertexIndex(2*i);poly.addVertexIndex(2*i+1);poly.addVertexIndex(2*nextIndex+1);poly.addVertexIndex(2*nextIndex);
		tmpMesh->addPoly(poly);
	}

	int middleVertexIndex = (int)tmpMesh->getVertexCount();
	tmpMesh->addVertex(localCoords + V3D(0, 0, length/2.0).rotate(rotAngle, rotAxis));
	tmpMesh->addVertex(localCoords + V3D(0, 0, -length/2.0).rotate(rotAngle, rotAxis));

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

	tmpMesh->computeNormals();
	meshes.push_back(tmpMesh);
}

void KS_MechanicalComponent::addCubeAroundApoint(const P3D & p, double d)
{
	//DynamicArray<P3D> tmpPointsList = getPoints_list();
	points_list.push_back(p);
	points_list.push_back(p + P3D(d, d, -d));
	points_list.push_back(p + P3D(d, d, d));
	points_list.push_back(p + P3D(d, -d, -d));
	points_list.push_back(p + P3D(d, -d, d));
	points_list.push_back(p + P3D(-d, d, -d));
	points_list.push_back(p + P3D(-d, d, d));
	points_list.push_back(p + P3D(-d, -d, -d));
	points_list.push_back(p + P3D(-d, -d, d));
	//setPoints_list(tmpPointsList);
}

void KS_MechanicalComponent::loadTriangleMeshFromFile(char* fName){
	GLMesh* tmpMesh = OBJReader::loadOBJFile(fName);
	tmpMesh->computeNormals();
	meshes.push_back(tmpMesh);
	Logger::print("loading mesh..");
}

/**
	This method draws the meshes of the component.
*/
void KS_MechanicalComponent::draw(){

	glPushMatrix();
//translation part
	glTranslated(position[0], position[1], position[2]);
	//and rotation part
	glRotated(DEG(gamma), n_gamma[0], n_gamma[1], n_gamma[2]);
	glRotated(DEG(beta), n_beta[0], n_beta[1], n_beta[2]);
	glRotated(DEG(alpha), n_alpha[0], n_alpha[1], n_alpha[2]);

	// now we'll draw the object's mesh
	for (uint i = 0; i < meshes.size(); i++) {
		meshes[i]->drawMesh();
	}

	glPopMatrix();
	


}


uint KS_MechanicalComponent::renderToObjFile(FILE* fp, uint vertexIdxOffset, double scale)
{
	//TransformationMatrix mat;
	//Transformation Tf;
	Quaternion quat = R_gamma*R_beta*R_alpha;

	//quat.getRotationMatrix(&mat);
	//Tf.R = quat.getRotationMatrix();

	//mat.setTranslation(this->position);
	//Tf.T = this->position;
	//mat.setScale(scale);

	uint retVal = 0;
	for (uint i=0;i<meshes.size();i++)
		//retVal += meshes[i]->renderToObjFile( fp, vertexIdxOffset + retVal, mat);
		retVal += meshes[i]->renderToObjFile(fp, vertexIdxOffset + retVal, quat, this->position);
	return retVal;
}

