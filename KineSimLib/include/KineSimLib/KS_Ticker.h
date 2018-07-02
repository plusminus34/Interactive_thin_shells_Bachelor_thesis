#pragma once

class KS_Ticker{
friend class KS_MechanicalAssembly;

public :

	KS_Ticker(){
		m_stepPhase=0;
		m_deltaStep = 1.0;
	}

	~KS_Ticker(){};

	void step(){
		m_stepPhase+=m_deltaStep;
	}

	void restart(){
		m_stepPhase=0;
	}

	double tick(){return m_stepPhase;}

	void setTickerValue(double v){
		m_stepPhase = v;
	}

private :
	double m_stepPhase;
public:
	double m_deltaStep;
};

