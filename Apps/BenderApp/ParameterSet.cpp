

#include <iostream>

#include "ParameterSet.h"



std::pair<double, double> ParameterSet::getParameterLimitsByLocalIdx(int idx)
{
	std::cerr << "Error: not implemented (" << __FILE__ << ":" << __LINE__ << std::endl; 
	exit(3);
}



void ParameterSet::pullVec(dVector & par_vec)
{
	par_vec.resize(getNPar());
	int i = 0;
	writeToList(par_vec, i);
}

void ParameterSet::pushVec(dVector & par_vec)
{
	int i = 0;
	setFromList(par_vec, i);
}