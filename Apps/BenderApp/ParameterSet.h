#pragma once


#include "MathLib/matrix.h"


class ParameterSet {

	// note: the abstract base class only provides an iterface; the actual data
	// is stored somewhere else (e.g. in a derived class).

public:
	int parametersStartIndex; // position of parameter set in context of the global parameter set


public:
	
	void pullVec(dVector & par_vec);
	void pushVec(dVector & par_vec);

	virtual void writeToList(dVector & par, int & cursor_idx_io) = 0;
	virtual void setFromList(dVector & par, int & cursor_idx_io) = 0;
	virtual int getNPar() const = 0;

};
