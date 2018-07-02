#pragma once
#include "KS_multilinkbar.h"

class KS_HermiteSplineLinkBar :
	public KS_MultiLinkBar
{
public:
	KS_HermiteSplineLinkBar(char* rName);
	~KS_HermiteSplineLinkBar(void);

	virtual bool loadFromFile(FILE* f);
	virtual bool writeToFile(FILE* f);

protected:
	
	int nSamples;
	dVector angles;
	dVector magnitudes;

	void generatePointsFromSpline();
};
