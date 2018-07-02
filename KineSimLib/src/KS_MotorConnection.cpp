#include "KineSimLib/KS_MotorConnection.h"
#include "KineSimLib/KS_MechanicalComponent.h"
#include "KineSimLib/KS_LoaderUtils.h"

KS_MotorConnection::KS_MotorConnection(void){
	p2pConstraint = NULL;
	pt2ptConstraint = NULL;
	v2vConstraint = NULL;
	m_offset=0;
	m_phase=0;
	m_sinPhaseOffset = 0;
	m_amplitude=PI/4;
	m_frequency=2*PI;
	m_frames=360;
	m_rate=1;
	m_ticker = NULL;
	m_rbf = NULL;
	isActivated=true;
	useRBF=false;
	useCSP=false;
	nOnC1 = Vector3d(0,0,1);
	nOnC2 = Vector3d(0,0,1);
	m_stepScale = 1.0;
}

KS_MotorConnection::~KS_MotorConnection(void){
	delete p2pConstraint;
	delete pt2ptConstraint;
	delete v2vConstraint;
}

void KS_MotorConnection::connect(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut){
	assignConnectedComponents(pCompIn, pCompOut);
	m_compIn = pCompIn;
	m_compOut = pCompOut;
	m_compOut->addCylinderMesh(20, 0.06, 0.08, pOnC2, nOnC2);
	m_pin=m_compOut->getNumOfMeshes()-1;
	if(isActivated) 
		m_compOut->getMesh(m_pin)->setColour(1.0,0.0,0.0,1.0);
	else
		m_compOut->getMesh(m_pin)->setColour(0.6,0.5,0.5,1.0);

	p2pConstraint = new KS_PhaseToPhaseConstraint(m_compIn, m_compOut, 1.0, m_phase);
	pt2ptConstraint = new KS_P2PConstraint(pOnC1, m_compIn, pOnC2, m_compOut); 
	v2vConstraint = new KS_V2VConstraint(nOnC1, m_compIn, nOnC2, m_compOut);
}

void KS_MotorConnection::addConstraintsToList(std::vector<KS_Constraint*>& constraints){
	if(isActivated)
		constraints.push_back(p2pConstraint);
	constraints.push_back(pt2ptConstraint);
	constraints.push_back(v2vConstraint);
}

void KS_MotorConnection::updateConnection(){
	double stepPhase = m_ticker->tick();
	if(useCSP){
		m_phase = stepPhase*m_amplitude/m_frames+m_offset;
	}else if(useRBF){
		stepPhase *= m_stepScale;
		m_phase = m_rbf->evaluate(int(stepPhase)%m_frames+stepPhase-int(stepPhase));
		//m_phase = motorPhaseAngle.evaluate_catmull_rom(int(stepPhase)%(m_frames)+stepPhase-int(stepPhase));
		//m_phase = motorPhaseAngle.evaluate_linear(int(stepPhase)%(m_frames)+stepPhase-int(stepPhase));
	}else{
		m_phase = m_amplitude*sin(2*PI*stepPhase/m_frequency + m_sinPhaseOffset)+m_offset;
		//m_phase = m_amplitude*sin(m_frequency*stepPhase/m_frames + m_sinPhaseOffset)+m_offset;
	}
	p2pConstraint->setOffset(m_phase);
}

double KS_MotorConnection::getPhase(double t, double tMax){
	if(useRBF){
		return m_rbf->evaluate(t*m_frames/tMax);
	}else if(useCSP){
		return t*m_amplitude/tMax+m_offset;
	}else{
		return m_amplitude*sin(2*PI*t/tMax+m_sinPhaseOffset)+m_offset;
	}
}

double KS_MotorConnection::getPhaseConstraintValue(){
	return p2pConstraint->getConstraintValues()->front();
}

bool KS_MotorConnection::loadFromFile(FILE* f, KS_MechanicalAssembly* ma){
	if (f == NULL){
		logPrint("KS_PhaseDriver: Cannot load input file.\n");
		return false;
	}
	//have a temporary buffer used to read the file line by line...
	char buffer[1000];
	DynamicArray<double> frames, angles;
	uint offset;
	//this is where it happens.
	while (!feof(f)){
		//get a line from the file...
		readValidLine(buffer, f, sizeof(buffer));
		char *line = lTrim(buffer);

		if (KS_Connection::processInputLine(line, ma))
			continue;

		int lineType = getKSLineType(line);
		switch (lineType) {
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
			case KS_IS_ACTIVATED:
				if (sscanf(line, "%d", &isActivated) != 1) assert(false);
				break;
			case KS_USE_SINUSOIDAL_SIGNAL:
				if (sscanf(line, "%lf %lf %lf %lf", &m_amplitude, &m_frequency, &m_offset, &m_sinPhaseOffset) < 3) assert(false);
				break;
			case KS_CONSTANT_SPEED_PROFILE:
				useCSP=true;
				if (sscanf(line," %lf %d %lf", &m_amplitude, &m_frames, &m_offset) !=3) assert(false);
				break;
			case KS_SCALE:
				sscanf(line, "%lf", &m_stepScale);
				break;
			case KS_NONCONSTANT_SPEED_PROFILE:
				if (sscanf(line, "%i %lf ", &m_frames, &m_rate) != 2) assert(false);
				useRBF=true;

				motorPhaseAngle.clear();


				frames.resize(m_frames+1);
				angles.resize(m_frames+1);
				offset=1;
				while((offset<strlen(line))&&(line[offset]!=' ')) offset++; 
				for(int i=0;i<m_frames;i++){
					frames[i]=i*m_rate;
					offset++;
					while((offset<strlen(line))&&(line[offset]!=' '&&line[offset]!='\t')) offset++; 
					if (sscanf(line+offset, "%lf", &angles[i]) != 1) assert(false);

					motorPhaseAngle.addKnot(frames[i], angles[i]);
				}

				motorPhaseAngle.addKnot(frames[m_frames-1]+1, angles[0]);

				frames[frames.size()-1] = m_frames*m_rate;
				angles[frames.size()-1] = angles[0];

				m_rbf = new RBF1D();
				m_rbf->setRBFData(frames,angles);
				m_frames=int(m_frames*m_rate);
				break;
			case KS_END:
				connect(this->m_compIn, this->m_compOut);
				return true;
				break;
			case KS_COMMENT:
				break;
			default:
				logPrint("Incorrect KS input file. Unexpected line: %s\n", buffer);
				return false;
		}
	}

	logPrint("Phase Driver: Warning - end of file met before END primitive\n");
	return false;
}


bool KS_MotorConnection::writeToFile(FILE* f){
	char* str;

	str = getKSString(KS_MOTOR_CON);
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

	str = getKSString(KS_IS_ACTIVATED);
	fprintf(f, "\t%s %d \n", str, isActivated);
	
	if(useRBF){
		str = getKSString(KS_NONCONSTANT_SPEED_PROFILE);
		fprintf(f, "\t%s %i %lf", str, int(m_frames/m_rate), m_rate);
		for(int i=0;i<m_frames/m_rate;i++)
			fprintf(f," %lf",m_rbf->evaluate(i*m_rate));
		fprintf(f,"\n");
		str = getKSString(KS_SCALE);
		fprintf(f, "\t%s %lf \n", str, m_stepScale);
	}else if(useCSP){
		str = getKSString(KS_CONSTANT_SPEED_PROFILE);
		fprintf(f, "\t%s %lf %d %lf\n", str, m_amplitude, m_frames, m_offset);
	}else{
		str = getKSString(KS_USE_SINUSOIDAL_SIGNAL);
		fprintf(f, "\t%s %lf %lf %lf\n", str, m_amplitude, m_frequency, m_offset);
	}

	str = getKSString(KS_END);
	fprintf(f, "%s\n\n\n", str);

	return true;
}

KS_MotorConnection* KS_MotorConnection::clone(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut, KS_Ticker* clock) const {
	KS_MotorConnection* pin = new KS_MotorConnection(*this);
	pin->m_compIn = pCompIn;
	pin->m_compOut = pCompOut;
	pin->p2pConstraint = p2pConstraint->clone(pCompIn,pCompOut);
	pin->pt2ptConstraint = pt2ptConstraint->clone(pCompIn,pCompOut);
	pin->v2vConstraint = v2vConstraint->clone(pCompIn,pCompOut);
	pin->linkWithTicker(clock);
	return pin;
}