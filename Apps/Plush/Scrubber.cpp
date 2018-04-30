#include "Scrubber.h"

Scrubber::Scrubber(int *i, int LENGTH) : Handler_v2() {
	this->i = i;
	this->LENGTH = LENGTH; 
}

bool Scrubber::character_event_(int key, int mods) {
	if (key == '.') {
		(*i)++;
		if (LENGTH != -1) {
			(*i) %= LENGTH;
		}
		return true;
	} else if (key == ',') {
		(*i)--;
		if (LENGTH != -1) {
			// while ((*i) < 0) { (*i) += LENGTH; }
			(*i) = max(0, (*i));
		}
		return true;
	}
	return false;
}
 