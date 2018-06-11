#pragma once

#include <Utils/Utils.h>
#include <ControlLib/GenericLimb.h>

class StepPattern{
public:
	//this is the limb - for fast access
	GenericLimb* limb;

	//keep track of the time index where the limb starts to 
	int startIndex;
	//and the relative phase when the limb should strikes the ground...
	int endIndex;

	StepPattern(GenericLimb* l, int start, int end){
		assert(start < end);
		this->limb = l;
		startIndex = start;
		endIndex = end;
	}
};

/**
	This class is used to parameterize a gait by specifying the typical foot-fall pattern. In particular, this class stores, for each foot of a character,
	the relative phase at which it is supposed to switch from stance to swing and the other way around.
*/
class FootFallPattern{
public:
	//keep track of the desired foot fall pattern for all feet...
	DynamicArray<StepPattern> stepPatterns;

	int strideSamplePoints;

	bool dirty = false;

public:
	FootFallPattern();
	~FootFallPattern(void);

	int getStepPatternIndexForLimb(GenericLimb* limb);
	StepPattern* getStepPatternForLimb(GenericLimb* limb);

	void addStepPattern(GenericLimb* limb, int start, int end);

	void doubleNumberOfSamples();

	bool isInStance(GenericLimb* limb, int tIndex){
		return !isInSwing(limb, tIndex);
	}

	bool isInStance(StepPattern* p, int tIndex){
		return !isInSwing(p, tIndex);
	}

	bool isAlwaysInSwing(GenericLimb* limb) {
		StepPattern* p = getStepPatternForLimb(limb);
		if (!p) 
			return false;
//		return (p->startIndex == 0 && p->endIndex == strideSamplePoints-1);
		return (p->endIndex - p->startIndex == strideSamplePoints - 1);
	}

	double getSwingPhaseForMotionPhase(GenericLimb* limb, double t) {
		boundToRange(t, 0, 0.9999999999);
		if (isAlwaysInSwing(limb))
			return 0.5;

		double intervalDuration = 1.0 / strideSamplePoints;

		double normalizedT = t / intervalDuration;

		//first, make sure index is bounded by strideEndIndex
		StepPattern* p = getStepPatternForLimb(limb);

		if (!p || !isInSwing(p, (int)normalizedT)) {
			return -1;
		}

		int startIndex = p->startIndex;
		int endIndex = p->endIndex;

		if ((int)normalizedT >= startIndex - strideSamplePoints && (int)normalizedT <= endIndex - strideSamplePoints)
			normalizedT += strideSamplePoints;

		if ((int)normalizedT >= startIndex + strideSamplePoints && (int)normalizedT <= endIndex + strideSamplePoints)
			normalizedT -= strideSamplePoints;

		return ((double)(normalizedT - startIndex)) / (endIndex + 1 - startIndex);
	}

	double getSwingPhaseForTimeIndex(GenericLimb* limb, int tIndex) {
		while (tIndex < 0) tIndex += strideSamplePoints;
		while (tIndex >= strideSamplePoints) tIndex -= strideSamplePoints;

		return getSwingPhaseForMotionPhase(limb, tIndex * 1.0 / strideSamplePoints);
	}

	bool isInSwing(GenericLimb* limb, int tIndex) {
		StepPattern* p = getStepPatternForLimb(limb);
		if (!p)
			return false;

		return isInSwing(p, tIndex);
	}

	bool isInSwing(StepPattern* p, int tIndex) {
		//first, make sure index is bounded by strideEndIndex
		while (tIndex < 0) tIndex+=strideSamplePoints;
		while (tIndex >=strideSamplePoints) tIndex -= strideSamplePoints;

		assert(tIndex >= 0 && tIndex < strideSamplePoints);

		int startIndex = p->startIndex;
		int endIndex = p->endIndex;

		//worry about wrap around and all that...
		return (tIndex >= startIndex && tIndex <= endIndex) || (tIndex >= startIndex - strideSamplePoints && tIndex <= endIndex - strideSamplePoints) || (tIndex >= startIndex + strideSamplePoints && tIndex <= endIndex + strideSamplePoints);
	}

	bool isStart(GenericLimb* limb, int tIndex){
		//first, make sure index is bounded by strideEndIndex
		while (tIndex < 0) tIndex+=strideSamplePoints;
		while (tIndex >=strideSamplePoints) tIndex -= strideSamplePoints;

		assert(tIndex >= 0 && tIndex < strideSamplePoints);

		StepPattern* p = getStepPatternForLimb(limb);

		int startIndex = p->startIndex;

		//worry about wrap around and all that...
		return tIndex == startIndex || tIndex == startIndex-strideSamplePoints || tIndex == startIndex+strideSamplePoints;
	}

	bool isEnd(GenericLimb* limb, int tIndex){
		//first, make sure index is bounded by strideEndIndex
		while (tIndex < 0) tIndex+=strideSamplePoints;
		while (tIndex >=strideSamplePoints) tIndex -= strideSamplePoints;

		assert(tIndex >= 0 && tIndex < strideSamplePoints);

		StepPattern* p = getStepPatternForLimb(limb);

		int endIndex = p->endIndex;

		//worry about wrap around and all that...
		return tIndex == endIndex || tIndex == endIndex-strideSamplePoints || tIndex == endIndex+strideSamplePoints;
	}

	void writeToFile(FILE* fp) {

		fprintf(fp, "%d %d\n\n", strideSamplePoints, stepPatterns.size());

		for (uint i = 0; i < stepPatterns.size(); i++)
			fprintf(fp, "%d %d\n", stepPatterns[i].startIndex, stepPatterns[i].endIndex);

	}

	void writeToFile(const char* fName){
		FILE* fp = fopen(fName, "w");

		writeToFile(fp);
		
		fclose(fp);
	}

	void loadFromFile(FILE* fp) {
		int tmpStrideSamplePoints = 0;
		int tmpStepPatterns = 0;
		fscanf(fp, "%d %d", &tmpStrideSamplePoints, &tmpStepPatterns);

		if (strideSamplePoints != tmpStrideSamplePoints || tmpStepPatterns != (int)stepPatterns.size()){
			Logger::consolePrint("FFP from file is incompatible with current settings. Skipping...\n");
			return;
		}

		for (uint i = 0; i < stepPatterns.size(); i++)
			fscanf(fp, "%d %d", &stepPatterns[i].startIndex, &stepPatterns[i].endIndex);
	}

	void loadFromFile(const char* fName){
		FILE* fp = fopen(fName, "r");
		if (!fp)
			return;

		loadFromFile(fp);
		
		fclose(fp);
	}

	bool isSameAs(FootFallPattern other) {
		if (this->strideSamplePoints != other.strideSamplePoints)
			return false;

		for (uint j = 0; j<stepPatterns.size(); j++)
			for (int i = 0; i < strideSamplePoints; i++) {
				bool thisSwing = this->isInSwing(this->stepPatterns[j].limb, i);
				bool otherSwing = other.isInSwing(this->stepPatterns[j].limb, i);
				if (thisSwing != otherSwing)
					return false;
			}
		return true;
	}


};


