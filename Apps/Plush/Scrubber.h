#pragma once
// --
#include <PlushHelpers/helpers_star.h>
// --
#include "Handler_v2.h"

class Scrubber : public Handler_v2 {

public:
	Scrubber(int *i, int LENGTH=-1);

	int *i = nullptr;
	int LENGTH = -1;

public:
	virtual bool character_event_(int key, int mods);

};
