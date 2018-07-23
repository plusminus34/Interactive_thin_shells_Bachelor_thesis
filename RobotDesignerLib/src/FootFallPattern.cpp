#include <RobotDesignerLib/FootFallPattern.h>
#include <MathLib/MathLib.h>

FootFallPattern::FootFallPattern(void){
	strideSamplePoints = 12;
}

FootFallPattern::~FootFallPattern(void){
}

int FootFallPattern::getStepPatternIndexForLimb(GenericLimb* limb) {
	for (uint i=0;i<stepPatterns.size();i++)
		if (stepPatterns[i].limb == limb)
			return i;

	return -1;
}

StepPattern* FootFallPattern::getStepPatternForLimb(GenericLimb* limb) {
	for (uint i=0;i<stepPatterns.size();i++)
		if (stepPatterns[i].limb == limb)
			return &stepPatterns[i];

	return NULL;
}

void FootFallPattern::addStepPattern(GenericLimb* limb, int start, int end){
	if (end < start)
		end += strideSamplePoints;
	stepPatterns.push_back(StepPattern(limb, start, end));
}

void FootFallPattern::doubleNumberOfSamples(){
	int oldSampleCount = strideSamplePoints;
	int newSampleCount = strideSamplePoints*2;
	strideSamplePoints = newSampleCount;

	for(uint i = 0; i < stepPatterns.size(); i++)
	{
		stepPatterns[i].startIndex *= 2;
		stepPatterns[i].endIndex = stepPatterns[i].endIndex*2+1;
	}
}


//this method assumes that the foot fall pattern (ffp) starts at time = 0 and then repeats forever. The time start then defines the appropriate point in the locomotion cycle that we should be starting from...
void ContinuousFootFallPattern::populateFromRepeatingFootFallPattern(FootFallPattern& ffp, double ffpDuration, double timeStart, double timeEnd) {
	for (uint i = 0; i < stepPatterns.size(); i++) {
		if (StepPattern* p = ffp.getStepPatternForLimb(stepPatterns[i].limb)) {
			int n = (int)(timeStart / ffpDuration); //this is the number of footfall patterns that would have elappsed since the start of the motion plan
			double t = n * ffpDuration;

			while (t < timeEnd) {
				stepPatterns[i].addSwingPhase(t + (double)p->startIndex / (ffp.strideSamplePoints) * ffpDuration, t + (double)(p->endIndex + 1) / (ffp.strideSamplePoints) * ffpDuration);
				t += ffpDuration;
			}
		}
	}
}

//this method assumes that the foot fall pattern (ffp) is synchronize to start with an offset phase at time = timeStart and then repeats forever.
void ContinuousFootFallPattern::populateFromRepeatingFootFallPatternWithTimeIndexOffset(FootFallPattern& ffp, double ffpDuration, int nStepsOffset, double timeStart, double timeEnd) {
	for (uint i = 0; i < stepPatterns.size(); i++) {
		if (StepPattern* p = ffp.getStepPatternForLimb(stepPatterns[i].limb)) {
			double t = timeStart - (nStepsOffset % ffp.strideSamplePoints) * ffpDuration / ffp.strideSamplePoints;

			while (t < timeEnd) {
				stepPatterns[i].addSwingPhase(t + (double)p->startIndex / (ffp.strideSamplePoints) * ffpDuration, t + (double)(p->endIndex + 1) / (ffp.strideSamplePoints) * ffpDuration);
				t += ffpDuration;
			}
		}
	}
}

