#pragma once


#include "MathLib/matrix.h"

#include <utility>


class ParameterSet {

	// note: the abstract base class only provides an iterface; the actual data
	// is stored somewhere else (e.g. in a derived class).

public:
	int parametersStartIndex; // position of parameter set in context of the global parameter set


public:
	// abstract interface
	virtual void writeToList(dVector & par, int & cursor_idx_io) = 0;
	virtual void setFromList(dVector & par, int & cursor_idx_io) = 0;
	virtual int getNPar() const = 0;
	virtual std::pair<double, double> getParameterLimitsByLocalIdx(int idx);

	// "wrappers" for easy access
	void pullVec(dVector & par_vec);
	void pushVec(dVector & par_vec);
	//virtual std::pair<double, double> getParameterLimitsByGlobalIdx(int idx) { return(getParameterLimitsByLocalIdx(idx - parametersStartIndex));}

};
