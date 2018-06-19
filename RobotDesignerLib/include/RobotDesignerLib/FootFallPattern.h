#pragma once

#include <Utils/Utils.h>
#include <ControlLib/GenericLimb.h>

class StepPattern{
public:
	//this is the limb - for fast access
	GenericLimb* limb;

	//keep track of the time index when the limb starts the swing phase
	int startIndex;
	//and when it should strike the ground again
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

class ContinuousStepPattern {
public:
	//this is the limb - for fast access
	GenericLimb* limb;
	RigidBody* eeRB;
	P3D eeLocalCoords;

	//keep track of the start and end time (in an absolute timeframe) for the swing phases (e.g. when the limb starts to lift off the ground, and when it strikes the ground again)
	DynamicArray<double> swingPhases;

	ContinuousStepPattern(GenericLimb* l, RigidBody* eeRB, const P3D& eeLocalCoords) {
		this->limb = l;
		this->eeRB = eeRB;
		this->eeLocalCoords = eeLocalCoords;
	}

	void addSwingPhase(double start, double end) {
		if (start > end) return;
		if (swingPhases.size() > 0 && start < swingPhases[swingPhases.size() - 1]) return;
		swingPhases.push_back(start);
		swingPhases.push_back(end);
	}

	double getFirstTimeInStanceAfter(double t) {
		if (isInStanceAt(t))
			return t;

		int i = getSwingPhaseIndexForTime(t);
		return swingPhases[2 * i + 1];
	}

	double getFirstTimeInSwingAfter(double t) {
		if (isInSwingAt(t))
			return t;

		uint i = 0;
		while (i < swingPhases.size()) {
			if (swingPhases[i] > t)
				return swingPhases[i];
			i += 2;
		}
		return -1;
	}

	int getSwingPhaseIndexForTime(double t) {
		uint i = 0;
		while (i < swingPhases.size()) {
			if (swingPhases[i] <= t && swingPhases[i + 1] >= t)
				return i/2;
			i += 2;
		}
		return -1;
	}

	double getDurationOfSwingPhaseAt(double t) {
		int i = getSwingPhaseIndexForTime(t);
		if (i < 0) return -1;

		return (swingPhases[2 * i + 1] - swingPhases[2 * i]);
	}

	double getSwingPhase(double t) {
		int i = getSwingPhaseIndexForTime(t);
		if (i < 0) return -1;
		return (t - swingPhases[2 * i]) / (swingPhases[2 * i + 1] - swingPhases[2 * i]);
	}

	double isInSwingAt(double t) {
		int i = getSwingPhaseIndexForTime(t);
		if (i >= 0)
			return true;
		return false;
	}

	double isInStanceAt(double t) {
		return !isInSwingAt(t);
	}

	void clearSwingPhasesBefore(double t) {
		while (swingPhases.size() > 0 && swingPhases[1] < t) {
			swingPhases.erase(swingPhases.begin());
			swingPhases.erase(swingPhases.begin());
		}
	}
};

class ContinuousFootFallPattern {
public:
	//keep track of the desired stepping pattern for each end effector...
	DynamicArray<ContinuousStepPattern> stepPatterns;

	void addStepPattern(GenericLimb* limb, RigidBody* eeRB, const P3D& eeLocalCoords) {
		stepPatterns.push_back(ContinuousStepPattern(limb, eeRB, eeLocalCoords));
	}

	void populateFrom(FootFallPattern& ffp, double ffpDuration, double timeStart, double timeEnd) {
		for (uint i = 0; i < stepPatterns.size(); i++) {
			if (StepPattern* p = ffp.getStepPatternForLimb(stepPatterns[i].limb)) {
				int n = (int)(timeStart / ffpDuration);
				double t = n * ffpDuration;
				while (t < timeEnd) {
					stepPatterns[i].addSwingPhase(t + (double)p->startIndex / (ffp.strideSamplePoints) * ffpDuration, t + (double)(p->endIndex+1) / (ffp.strideSamplePoints) * ffpDuration);
					t += ffpDuration;
				}
			}
		}
	}
};

