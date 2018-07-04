#include "KineSimLib/KS_BindComponentsConnection.h"
#include "KineSimLib/KS_LoaderUtils.h"


KS_BindComponentsConnection::KS_BindComponentsConnection(void){
	p2pConstraint = NULL;
	v2vConstraint = v2vConstraintForWelding = NULL;
	nOnC1 = V3D(0,0,1);
	nOnC2 = V3D(0,0,1);
	weldComponents = false;
	allowArbitraryRelativeRotation = false;

}

KS_BindComponentsConnection::~KS_BindComponentsConnection(void){
	delete p2pConstraint;
	delete v2vConstraint;
	delete v2vConstraintForWelding;
}

void KS_BindComponentsConnection::connect(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut){
	assignConnectedComponents(pCompIn, pCompOut);
	m_compIn = pCompIn;
	m_compOut = pCompOut;
/*
	double spacing = 0.3;//mm
	double scale = 38.0 / pinJointScale;


	V3D worldPinPos = m_compIn->get_w(pOnC1);
	logPrint3("Connection between %s and %s\n", m_compIn->getName(), m_compOut->getName());
	logPrint3("\tWorld coords of pin joint: %lf %lf %lf\n", worldPinPos.x(), worldPinPos.y(), worldPinPos.z());
	V3D localPinPos = m_compOut->get_x(worldPinPos);
	logPrint3("\tLocal coords of pin joint: %lf %lf %lf\n", localPinPos.x(), localPinPos.y(), localPinPos.z());
	
	bool createJunctions = false;
/ *
	if(createJunctions)
	{
		//reinforce the components a bit...
		m_compIn->addBump(V3D(pOnC1.x(), pOnC1.y(), 0), 5.0/scale, m_compIn->getThickness(), nOnC1);
		m_compOut->addBump(V3D(pOnC2.x(), pOnC2.y(), 0), 5.0/scale, m_compOut->getThickness(), nOnC2);

		m_compIn->addBump(pOnC1, (3.0+spacing)/scale, (m_compOut->getThickness() + m_compIn->getThickness()) * 1.3 + 3 * spacing/scale, nOnC1); //outer
		m_compIn->addBump(pOnC1, 3.0/scale, (m_compOut->getThickness() + m_compIn->getThickness()) * 1.3 + 3 * spacing/scale, nOnC1); //inner

		m_compIn->addBump(pOnC1 + V3D(0,0,1) * ((m_compOut->getThickness() + m_compIn->getThickness())/2 + 1.5 * spacing/scale + 0.025/2), 5.0/scale, 2.0/scale, nOnC1); //end stop 1
		m_compIn->addBump(pOnC1 - V3D(0,0,1) * ((m_compOut->getThickness() + m_compIn->getThickness())/2 + 1.5 * spacing/scale + 0.025/2), 5.0/scale, 2.0/scale, nOnC1); //end stop 1

		if (createPinJoint){
			m_compIn->addBump(pOnC1, 0.03, 0.4, nOnC1);
			m_compOut->addBump(pOnC2, 0.03, 0.4, nOnC2);
		}
	}
* /
//	m_compIn->addBump(pOnC1, 0.03 * pinJointScale, 0.3, nOnC1);
//	m_compOut->addBump(pOnC2, 0.03 * pinJointScale, 0.3, nOnC2);
* /
*/
	m_compOut->addCylinderMesh(20, 0.03, 0.08, pOnC2, nOnC2);
	m_pin=m_compOut->getNumOfMeshes()-1;
	//m_compOut->getMesh(m_pin)->setColour(0.0,0.0,0.0,1.0);

	p2pConstraint = new KS_P2PConstraint(pOnC1, m_compIn, pOnC2, m_compOut); 
	if (allowArbitraryRelativeRotation == false){
		v2vConstraint = new KS_V2VConstraint(nOnC1, m_compIn, nOnC2, m_compOut); 
		if (weldComponents){
			//need to come up with another set of vectors, one on each component, that we need to constrain so that they are aligned
			V3D n2OnC1 = V3D(1,0,0), n2OnC2;
			if (fabs(n2OnC1.dot(nOnC1)) > 0.95) n2OnC1 = V3D(1,1,1);
			n2OnC1 -= nOnC1 * (n2OnC1.dot(nOnC1)); n2OnC1.normalize();
			//now n2OnC1 should be perpendicular to nOnC1
			assert(fabs(n2OnC1.dot(nOnC1)) < 0.001);
			//we need to find a corresponding vector on c2 now...
			V3D rotAxis = nOnC1.cross(nOnC2);
			if (rotAxis.length() < 0.0001){ //nOnC1 is the same as nOnC2
				n2OnC2 = n2OnC1;
				n2OnC2 -= nOnC2 * (n2OnC2.dot(nOnC2)); 
				n2OnC2.normalize();
			}else{
				rotAxis.normalize();
				double angle = nOnC1.angleWith(nOnC2);
				n2OnC2 = n2OnC1.rotate(angle, rotAxis);//using rotate function from the V3D class not from Qauternion
				n2OnC2 -= nOnC2 * (n2OnC2.dot(nOnC2)); 
				n2OnC2.normalize();
			}
			//now n2OnC2 should be perpendicular to nOnC2
			assert(fabs(n2OnC2.dot(nOnC2)) < 0.001);
			v2vConstraintForWelding = new KS_V2VConstraint(n2OnC1, m_compIn, n2OnC2, m_compOut); 
		}
	}
}

bool KS_BindComponentsConnection::loadFromFile(FILE* f, KS_MechanicalAssembly* ma){
	if (f == NULL){
		Logger::print("KS_BindComponentsConnection: Cannot load input file.\n");
		return false;
	}
	//have a temporary buffer used to read the file line by line...
	char buffer[200];
	//this is where it happens.
	while (!feof(f)){
		//get a line from the file...
		readValidLine(buffer, f, sizeof(buffer));
		char *line = lTrim(buffer);

		if (KS_Connection::processInputLine(line, ma))
			continue;

		int lineType = getKSLineType(line);
		switch (lineType) {
			case KS_COMMENT:
				break;
			case KS_END:
				connect(this->m_compIn, this->m_compOut);
				return true;
				break;
			case KS_PIN_ON_COMP_IN:
				if (sscanf(line, "%lf %lf %lf", &pOnC1[0], &pOnC1[1], &pOnC1[2]) != 3) assert(false);
				break;
			case KS_PIN_ON_COMP_OUT:
				if (sscanf(line, "%lf %lf %lf", &pOnC2[0], &pOnC2[1], &pOnC2[2]) != 3) assert(false);
				break;
			case KS_VEC_ON_COMP_IN:
				if (sscanf(line, "%lf %lf %lf", &nOnC1[0], &nOnC1[1], &nOnC1[2]) != 3) assert(false);
				nOnC1.normalize();
				break;
			case KS_VEC_ON_COMP_OUT:
				if (sscanf(line, "%lf %lf %lf", &nOnC2[0], &nOnC2[1], &nOnC2[2]) != 3) assert(false);
				nOnC2.normalize();
				break;
			case KS_WELD_COMPONENTS:
				weldComponents = true;
				break;
			case KS_ALLOW_ARBITRARY_ROTATION:
				allowArbitraryRelativeRotation = true;
				break;
			default:
				Logger::print("Incorrect KS input file. Unexpected line: %s\n", buffer);
				return false;
		}
	}

	Logger::print("KS_BindComponentsConnection: Warning - end of file met before END primitive\n");
	return false;
}

bool KS_BindComponentsConnection::writeToFile(FILE* f){

	char* str;

	str = getKSString(KS_BIND_COMPONENTS_CON);
	fprintf(f, "%s\n", str);

	writeBaseConnectionToFile(f);

	str = getKSString(KS_PIN_ON_COMP_IN);
	fprintf(f, "\t%s %lf %lf %lf\n", str, pOnC1[0], pOnC1[1], pOnC1[2]);

	str = getKSString(KS_PIN_ON_COMP_OUT);
	fprintf(f, "\t%s %lf %lf %lf\n", str, pOnC2[0], pOnC2[1], pOnC2[2]);

	str = getKSString(KS_VEC_ON_COMP_IN);
	fprintf(f, "\t%s %lf %lf %lf\n", str, nOnC1[0], nOnC1[1], nOnC1[2]);

	str = getKSString(KS_VEC_ON_COMP_OUT);
	fprintf(f, "\t%s %lf %lf %lf\n", str, nOnC2[0], nOnC2[1], nOnC2[2]);

	if (weldComponents){
		str = getKSString(KS_WELD_COMPONENTS);
		fprintf(f, "\t%s\n", str);
	}

	if (allowArbitraryRelativeRotation){
		str = getKSString(KS_ALLOW_ARBITRARY_ROTATION);
		fprintf(f, "\t%s\n", str);
	}

	str = getKSString(KS_END);
	fprintf(f, "%s\n\n\n", str);

	return true;
}

KS_BindComponentsConnection* KS_BindComponentsConnection::clone(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut, KS_Ticker* clock) const {
	KS_BindComponentsConnection* pin = new KS_BindComponentsConnection(*this);
	pin->m_compIn = pCompIn;
	pin->m_compOut = pCompOut;
	pin->p2pConstraint = p2pConstraint->clone(pCompIn,pCompOut);
	if(allowArbitraryRelativeRotation == false){
		pin->v2vConstraint = v2vConstraint->clone(pCompIn,pCompOut); 
		if(weldComponents){
			pin->v2vConstraintForWelding = v2vConstraintForWelding->clone(pCompIn,pCompOut); 
		}
	}
	return pin;
}