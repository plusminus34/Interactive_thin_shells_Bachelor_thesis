#include "KineSimLib/KS_PhaseDriverConnection.h"
#include "KineSimLib/KS_LoaderUtils.h"

KS_PhaseDriverConnection::KS_PhaseDriverConnection(KS_Ticker* ticker){
	this->m_ticker = ticker;
	useSinuosoidalSignal = false;
	useNonConstantSpeedProfile = false;
	angleStep = PI/32.0;//0.01;
	offset = 0;
	frequency = 0;
	phaseOffset = 0;
}

void KS_PhaseDriverConnection::updateConnection(){
	//we connect components directly to the ticker in two ways:
		//alpha = angleStep * tickerCount + offset
		//or
		//alpha = amplitude * sin(frequency * tickerCount + phaseOffset) + offset
	double phase = 0;
	if (useSinuosoidalSignal)
		phase = amplitude * sin(frequency * m_ticker->tick() + phaseOffset) + offset;
	if (useNonConstantSpeedProfile){
		phase = angleStep * m_ticker->tick() + offset;
		//now map this using the potentially non constant angular speed of the driver...
		double t = phase / (2*PI);
		int extraRevolutions = (int)floor(t);
		if (t <= 0 || t >= 1) t -= extraRevolutions;
		phase = (nonConstantSpeedProfile.evaluate(t) + extraRevolutions) * 2 * PI;
	}else
		phase = angleStep * m_ticker->tick() + offset;

	phaseConstraint->setTargetPhase(phase);
}


KS_PhaseDriverConnection::~KS_PhaseDriverConnection(void){
	delete phaseConstraint;
}

void KS_PhaseDriverConnection::connect(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut){
	assignConnectedComponents(pCompIn, pCompOut);
	m_compIn = pCompIn;
	m_compOut = pCompOut;
	assert(m_compIn==NULL);
	assert(m_ticker!=NULL);
	
	phaseConstraint = new KS_PhaseConstraint(m_compOut);
}

bool KS_PhaseDriverConnection::loadFromFile(FILE* f, KS_MechanicalAssembly* ma){
	if (f == NULL){
		logPrint("KS_PhaseDriverConnection: Cannot load input file.\n");
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
			case KS_SCALE:
				sscanf(line, "%lf", &angleStep);
				break;
			case KS_PHASE_INITIAL_OFFSET:
				sscanf(line, "%lf", &offset);
				break;
			case KS_USE_SINUSOIDAL_SIGNAL:
				sscanf(line, "%lf %lf %lf %lf", &amplitude, &frequency, &phaseOffset, &offset);
				useSinuosoidalSignal = true;
				break;
			case KS_NONCONSTANT_SPEED_PROFILE:
				{
					double f1 = 0.2, f2 = 0.4, f3 = 0.6, f4 = 0.8;
					if (sscanf(line, "%lf %lf %lf %lf", &f1, &f2, &f3, &f4) != 4) assert(false);
					nonConstantSpeedProfile.setRBF1DFunctionData(f1, f2, f3, f4);
					useNonConstantSpeedProfile = true;
				}
				break;
			case KS_END:
				connect(this->m_compIn, this->m_compOut);
				return true;
				break;
			default:
				logPrint("Incorrect KS input file. Unexpected line: %s\n", buffer);
				return false;
		}
	}

	logPrint("KS_PhaseDriverConnection: Warning - end of file met before END primitive\n");
	return false;
}

bool KS_PhaseDriverConnection::writeToFile(FILE* f){
	char* str;

	str = getKSString(KS_PHASE_DRIVER_CON);
	fprintf(f, "%s\n", str);

	writeBaseConnectionToFile(f);

	if (useSinuosoidalSignal){
		str = getKSString(KS_USE_SINUSOIDAL_SIGNAL);
		fprintf(f, "\t%s %lf %lf %lf %lf\n", str, amplitude, frequency, phaseOffset, offset);
	}
	else{
		str = getKSString(KS_SCALE);
		fprintf(f, "\t%s %lf\n", str, angleStep);

		str = getKSString(KS_PHASE_INITIAL_OFFSET);
		fprintf(f, "\t%s %lf\n", str, offset);

		if (useNonConstantSpeedProfile)
			fprintf(f, "\t%s %lf %lf %lf %lf\n", getKSString(KS_NONCONSTANT_SPEED_PROFILE), nonConstantSpeedProfile.b[1], nonConstantSpeedProfile.b[2], nonConstantSpeedProfile.b[3], nonConstantSpeedProfile.b[4]);
	}	

	str = getKSString(KS_END);
	fprintf(f, "%s\n\n\n", str);

	return true;
}

KS_PhaseDriverConnection* KS_PhaseDriverConnection::clone(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut, KS_Ticker* clock) const {
	KS_PhaseDriverConnection* pin = new KS_PhaseDriverConnection(*this);
	pin->connect(pCompIn,pCompOut);
	pin->m_ticker=clock;
	return pin;
}