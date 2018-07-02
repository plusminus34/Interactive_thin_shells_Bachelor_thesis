#include "KineSimLib/KS_Gear.h"
#include "KineSimLib/KS_LoaderUtils.h"

#define LOCAL_COORDINATES_GEAR_AXIS (getAlphaAxis())

KS_Gear::KS_Gear(char* gearName) : KS_MechanicalComponent(gearName){
	setSpurGearParameters(0.1, 0.5, 0, 20, RAD(20), 0.07);
	meshOffsetAngle = 0;
	phaseDriverOffset = 0.0;
}

KS_Gear* KS_Gear::clone() const {
	KS_Gear* comp = new KS_Gear(*this);
	for(uint i=0;i<meshes.size();i++)
		comp->meshes[i]=this->meshes[i]->clone();
	for(uint i=0;i<this->tracerParticles.size();i++)
		comp->tracerParticles[i].mc=dynamic_cast<KS_MechanicalComponent*>(comp);
	return comp;
}

KS_Gear::~KS_Gear(void){
}

void KS_Gear::setSpurGearParameters(double pthickness, double pPitchRadius, double pPitchSurfaceAngle, int pNumberOfTeeth, double pPressureAngle, double pTeethHeight){
	this->thickness = pthickness;
	this->pitchRadius = pPitchRadius;
	this->numberOfTeeth = pNumberOfTeeth;
	this->pressureAngle = pPressureAngle;
	this->teethHeight = pTeethHeight;
	this->pitchSurfaceAngle = pPitchSurfaceAngle;

}

void KS_Gear::setupGeometry(){
	GLMesh* tmpMesh = new GLMesh();

	//now move on to the geometry of the teeth
	double angleBetweenTeeth = 2 * PI / numberOfTeeth;
	//distance along the pitch circle
	double dedendumToAddendumRatio = 1.05;
	//radial distance from the pitch surface to the outermost point of the tooth
	double addendum = 1.0 * teethHeight / (1+dedendumToAddendumRatio);
	//radial distance from the depth of the tooth trough to the pitch surface
	double dedendum = dedendumToAddendumRatio * teethHeight / (1+dedendumToAddendumRatio);
	//radius of the KS_Gear, measured at the top of the teeth
	double outsideRadius = pitchRadius + addendum;
	//diameter of the KS_Gear, measured at the base of the teeth
	double rootRadius = pitchRadius - dedendum;

	int NUMBER_OF_VERTICES_PER_TOOTH_FACE = 6;

	double toothThicknessAsFunctionOfPitchDistance = 0.45;
		
	for (int i = 0; i < numberOfTeeth; i++){
		double toothAngle = i * angleBetweenTeeth;
		//obtain the profile of the root at the pitch circle. This will be then rotated according to the pitch angle and extruded to create the full tooth
		Vector3d pitchToRoot = Vector3d(cos(toothAngle) * (pitchRadius - rootRadius), sin(toothAngle) * (pitchRadius - rootRadius), 0);
		Vector3d pitchToOutside = Vector3d(cos(toothAngle) * (pitchRadius - outsideRadius), sin(toothAngle) * (pitchRadius - outsideRadius), 0);

		//the sloped edges of the tooth must pass through these points which are on the pitch diameter, no matter what the pressure angle is
		Vector3d midToothPitchPoint(cos(toothAngle) * pitchRadius, sin(toothAngle) * pitchRadius, 0);
		Vector3d leftToothPitchPoint(cos((i - toothThicknessAsFunctionOfPitchDistance/2.0) * angleBetweenTeeth) * pitchRadius, sin((i - toothThicknessAsFunctionOfPitchDistance/2.0) * angleBetweenTeeth) * pitchRadius, 0);
		Vector3d rightToothPitchPoint(cos((i + toothThicknessAsFunctionOfPitchDistance/2.0) * angleBetweenTeeth) * pitchRadius, sin((i + toothThicknessAsFunctionOfPitchDistance/2.0) * angleBetweenTeeth) * pitchRadius, 0);

		Vector3d rotationAxis = Vector3d(midToothPitchPoint).cross(LOCAL_COORDINATES_GEAR_AXIS);
		rotationAxis.normalize();
		Vector3d toothFaceNormal = rotate(LOCAL_COORDINATES_GEAR_AXIS, pitchSurfaceAngle, rotationAxis);

		//now we know where each side of the tooth starts and ends...
		Vector3d leftToothStart = leftToothPitchPoint - rotate(rotate(pitchToRoot, pressureAngle * 0, LOCAL_COORDINATES_GEAR_AXIS), pitchSurfaceAngle, rotationAxis);
		Vector3d leftToothEnd = leftToothPitchPoint - rotate(rotate(pitchToOutside, pressureAngle, LOCAL_COORDINATES_GEAR_AXIS), pitchSurfaceAngle, rotationAxis);
		Vector3d rightToothStart = rightToothPitchPoint - rotate(rotate(pitchToRoot, -pressureAngle * 0, LOCAL_COORDINATES_GEAR_AXIS), pitchSurfaceAngle, rotationAxis);
		Vector3d rightToothEnd = rightToothPitchPoint - rotate(rotate(pitchToOutside, -pressureAngle, LOCAL_COORDINATES_GEAR_AXIS), pitchSurfaceAngle, rotationAxis);

		int start = (int)tmpMesh->getVertexCount();
		for (int j=0;j<2;j++){
			int multiplier = 1;
			if (j == 1) multiplier = -1;
			tmpMesh->addVertex(Point3d() + (leftToothStart + toothFaceNormal * thickness / 2.0 * multiplier));
			tmpMesh->addVertex(Point3d() + (leftToothPitchPoint + toothFaceNormal * thickness / 2.0* multiplier));
			tmpMesh->addVertex(Point3d() + (leftToothEnd + toothFaceNormal * thickness / 2.0* multiplier));
			tmpMesh->addVertex(Point3d() + (rightToothEnd + toothFaceNormal * thickness / 2.0* multiplier));
			tmpMesh->addVertex(Point3d() + (rightToothPitchPoint + toothFaceNormal * thickness / 2.0* multiplier));
			tmpMesh->addVertex(Point3d() + (rightToothStart + toothFaceNormal * thickness / 2.0 * multiplier));

			tmpMesh->addPoly(GLIndexedQuad(start + 0 + j*NUMBER_OF_VERTICES_PER_TOOTH_FACE, start + 1 + j*NUMBER_OF_VERTICES_PER_TOOTH_FACE, start + 4 + j*NUMBER_OF_VERTICES_PER_TOOTH_FACE, start + 5 + j*NUMBER_OF_VERTICES_PER_TOOTH_FACE, j==1));
			tmpMesh->addPoly(GLIndexedQuad(start + 1 + j*NUMBER_OF_VERTICES_PER_TOOTH_FACE, start + 2 + j*NUMBER_OF_VERTICES_PER_TOOTH_FACE, start + 3 + j*NUMBER_OF_VERTICES_PER_TOOTH_FACE, start + 4 + j*NUMBER_OF_VERTICES_PER_TOOTH_FACE, j==1));
		}
		//connect the front and back side of the teeth
		for (int j=0;j<NUMBER_OF_VERTICES_PER_TOOTH_FACE-1;j++)
			tmpMesh->addPoly(GLIndexedQuad(start + j, start + NUMBER_OF_VERTICES_PER_TOOTH_FACE + j, start + NUMBER_OF_VERTICES_PER_TOOTH_FACE + j+1, start + j+1));
	}


	//geometry for the teeth is now generated, but we need to close the gaps and fill in the faces of the KS_Gear...
	for (int i = 0; i < numberOfTeeth; i++){
		int tmpI = i-1;
		if (i == 0) tmpI = numberOfTeeth-1;
		tmpMesh->addPoly(GLIndexedQuad(2 * NUMBER_OF_VERTICES_PER_TOOTH_FACE * i + 0, 2 * NUMBER_OF_VERTICES_PER_TOOTH_FACE * i + NUMBER_OF_VERTICES_PER_TOOTH_FACE + 0, 2 * NUMBER_OF_VERTICES_PER_TOOTH_FACE * tmpI + NUMBER_OF_VERTICES_PER_TOOTH_FACE + 5, 2 * NUMBER_OF_VERTICES_PER_TOOTH_FACE * tmpI + 5, true));
	}

	//and now the faces...
	
	for (int j=0;j<2;j++){
		int offset = 0;
		if (j == 1) offset = NUMBER_OF_VERTICES_PER_TOOTH_FACE;

		//these next two points are the centers of the two faces of the KS_Gear. They are placed so that the face of the KS_Gear is normal to its pitch surface at any point...
		Vector3d v1, v2;
		double t;
		v1 = Vector3d(tmpMesh->getVertex(0 + offset));
		v2 = Vector3d(tmpMesh->getVertex(0+NUMBER_OF_VERTICES_PER_TOOTH_FACE) - tmpMesh->getVertex(0));
		t = (v1.x*v2.x + v1.y*v2.y) / v2.z + tmpMesh->getVertex(0 + offset).z;
		if (j == 0) 
			boundToRange(&t, tmpMesh->getVertex(0 + offset).z, thickness);
		else
			boundToRange(&t, -thickness, tmpMesh->getVertex(0 + offset).z);
		tmpMesh->addVertex(Point3d(0, 0, t));	
		for (int i = 0; i < numberOfTeeth; i++){
			int tmpI = i-1;
			if (i == 0) tmpI = numberOfTeeth-1;
			tmpMesh->addPoly(GLIndexedTriangle(2 * NUMBER_OF_VERTICES_PER_TOOTH_FACE * tmpI + 5 + offset, 2 * NUMBER_OF_VERTICES_PER_TOOTH_FACE * i + offset + 0, (int)tmpMesh->getVertexCount()-1, j==1));
			tmpMesh->addPoly(GLIndexedTriangle(2 * NUMBER_OF_VERTICES_PER_TOOTH_FACE * i + 0 + offset, 2 * NUMBER_OF_VERTICES_PER_TOOTH_FACE * i + 5 + offset, (int)tmpMesh->getVertexCount()-1, j==1));
		}
	}

	for (int i=0; i<tmpMesh->getVertexCount();i++)
		tmpMesh->setVertex(i, Point3d() + rotate(Vector3d(tmpMesh->getVertex(i)), meshOffsetAngle, LOCAL_COORDINATES_GEAR_AXIS));

	tmpMesh->computeNormals();
	tmpMesh->setColour(meshColor[0], meshColor[1], meshColor[2], 1);
	meshes.push_back(tmpMesh);
}

bool KS_Gear::writeToFile(FILE* f){
	char* str;

	str = getKSString(KS_SPUR_GEAR);
	fprintf(f, "%s\n", str);

	writeBaseComponentToFile(f);

	str = getKSString(KS_THICKNESS);
	fprintf(f, "\t%s %lf\n", str, thickness);

	str = getKSString(KS_GEAR_MESH_OFFSET_ANGLE);
	fprintf(f, "\t%s %lf\n", str, meshOffsetAngle);

	str = getKSString(KS_RADIUS);
	fprintf(f, "\t%s %lf\n", str, pitchRadius);

	str = getKSString(KS_PITCH_SURFACE_ANGLE);
	fprintf(f, "\t%s %lf\n", str, pitchSurfaceAngle);

	str = getKSString(KS_NUMBER_OF_TEETH);
	fprintf(f, "\t%s %d\n", str, numberOfTeeth);

	str = getKSString(KS_TEETH_HEIGHT);
	fprintf(f, "\t%s %lf\n", str, teethHeight);

	str = getKSString(KS_PRESSURE_ANGLE);
	fprintf(f, "\t%s %lf\n", str, pressureAngle);

	str = getKSString(KS_GEAR_PHASE_DRIVER_OFFSET);
	fprintf(f, "\t%s %lf\n", str, phaseDriverOffset);

	str = getKSString(KS_END);
	fprintf(f, "%s\n\n\n", str);

	return true;
}
bool KS_Gear::loadFromFile(FILE* f){
	if (f == NULL){
		logPrint("KS_Gear: Cannot load input file.\n");
		return false;
	}
	//have a temporary buffer used to read the file line by line...
	char buffer[200];
	//this is where it happens.
	while (!feof(f)){
		//get a line from the file...
		readValidLine(buffer, f, sizeof(buffer));
		char *line = lTrim(buffer);

		if (KS_MechanicalComponent::processInputLine(line))
			continue;

		int lineType = getKSLineType(line);
		switch (lineType) {
			case KS_THICKNESS:
				if(sscanf(line, "%lf", &thickness) != 1) assert(false);
				break;
			case KS_GEAR_MESH_OFFSET_ANGLE:
				if(sscanf(line, "%lf", &meshOffsetAngle) != 1) assert(false);
				break;
			case KS_RADIUS:
				if(sscanf(line, "%lf", &pitchRadius) != 1) assert(false);
				break;
			case KS_PITCH_SURFACE_ANGLE:
				if(sscanf(line, "%lf", &pitchSurfaceAngle) != 1) assert(false);
				break;
			case KS_NUMBER_OF_TEETH:
				if(sscanf(line, "%d", &numberOfTeeth) != 1) assert(false);
				break;
			
			case KS_TEETH_HEIGHT:
				if(sscanf(line, "%lf", &teethHeight) != 1) assert(false);
				break;
			case KS_PRESSURE_ANGLE:
				if(sscanf(line, "%lf", &pressureAngle) != 1) assert(false);
				break;
			case KS_NOT_IMPORTANT:
				logPrint("KS_Gear warning: Ignoring input line: \'%s\'\n", line);
				break;
			case KS_COMMENT:
				break;
			case KS_END:
				setupGeometry();
				return true;
				break;
			case KS_GEAR_PHASE_DRIVER_OFFSET:
				if(sscanf(line, "%lf", &phaseDriverOffset) != 1) assert(false);
				break;
			default:
				logPrint("Incorrect KS input file. Unexpected line: %s\n", buffer);
				return false;
		}
	}

	logPrint("KS_Gear: Warning - end of file met before END primitive\n");
	return false;
}


void KS_Gear::setPressureAngle( double pressureAngle )
{
	this->pressureAngle = pressureAngle;
}

void KS_Gear::setMeshOffsetAngle( double meshOffsetAngle )
{
	this->meshOffsetAngle = meshOffsetAngle;
}

void KS_Gear::setPitchSurfaceAngle( double pitchSurfaceAngle )
{
	this->pitchSurfaceAngle = pitchSurfaceAngle;
}
