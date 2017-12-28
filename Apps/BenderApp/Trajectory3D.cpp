

#include <GUILib/GLUtils.h>

#include "Trajectory3D.h"




Trajectory3Dplus::~Trajectory3Dplus() {
		delete discreteSpline;
}


void Trajectory3Dplus::getMinDistanceLinear(P3D const & pt, double & d, int & i, bool & within_segment) const
{
	within_segment = false;

	if(values.size() < 1) {
		d = 0;
		i = -1;
		return;
	}
	if(values.size() == 1) {
		V3D pt02pt(values[0], pt);
		d = std::sqrt(pt02pt.dot(pt02pt));
		i = 0;
		return;
	}

	i = -1;
	double d2_min = std::numeric_limits<double>::max();
	for(int j = 0; j < values.size()-1; ++j) {
		V3D ptj2pt(values[j], pt);
		V3D ptj2ptjp1(values[j], values[j+1]);
		double prod = ptj2pt.dot(ptj2ptjp1);
		// compute distance to line segment
		int i_temp;
		double d2_temp;
		if(prod <= 0) {	// "before" segment
			i_temp = j;
			d2_temp = ptj2pt.dot(ptj2pt);
		}
		else if(prod < ptj2ptjp1.dot(ptj2ptjp1)) {	//"within" segment
			within_segment = true;
			i_temp = j;
			d2_temp = ptj2pt.cross(ptj2ptjp1).squaredNorm() / ptj2ptjp1.squaredNorm();
		}
		else {	// "after" segment
			i_temp = j+1;
			V3D ptjp12pt(values[j+1], pt);
			d2_temp = ptjp12pt.squaredNorm();
		}
		// assign
		if(d2_temp < d2_min) {
			d2_min = d2_temp;
			i = i_temp;
		}
	}
	
	d = std::sqrt(d2_min);
}


void Trajectory3Dplus::updateTvalues() {
	
	tValues.resize(values.size());

	if(values.size() < 1) {
		return;
	}

	tValues[0] = 0.0;
	for(int i = 1; i < values.size(); ++i) {
		tValues[i] = tValues[i-1] + V3D(values[i-1], values[i]).length();
	}
}

void Trajectory3Dplus::updateDiscreteSpline() {

	if(discreteSpline == NULL) {
		discreteSpline = new Trajectory3Dplus();
	}

	createDiscreteSpline(50, *discreteSpline);

}


void Trajectory3Dplus::createDiscreteSpline(int n, Trajectory3Dplus & spline) {
	
	assert(n > 1);

	double t_tot = this->getMaxPosition() - this->getMinPosition();
	double dt = t_tot / static_cast<double>(n-1);

	spline.values.resize(n);
	spline.tValues.resize(n);
	// first and last knot of spline
	spline.tValues.front() = this->getMinPosition();
	spline.tValues.back() = this->getMaxPosition();
	spline.values.front() = this->values.front();
	spline.values.back() = this->values.back();
	// rest
	for(int i = 1; i < n-1; ++i) {
		spline.tValues[i] = this->getMinPosition() + dt * static_cast<double>(i);
		spline.values[i] = this->evaluate_catmull_rom(spline.tValues[i], false);
	}
}



void Trajectory3Dplus::addKnotInteractive(P3D const & pt) {

	double distance;
	int segmentID;
	bool is_within_segment;

	// find spot to insert knot
	getMinDistanceLinear(pt, distance, segmentID, is_within_segment);

	if(is_within_segment && distance < V3D(values[segmentID], values[segmentID+1]).length()) {
		// insert a knot here
		values.insert(values.begin()+segmentID+1, pt);
	}
	else if(!is_within_segment && segmentID == 0) {
		// add knot to start
		values.insert(values.begin(), pt);
	}
	else if(!is_within_segment && segmentID == values.size()-1) {
		// add knot to end
		values.push_back(pt);
	}

	updateTvalues();
	updateDiscreteSpline();

}



void Trajectory3Dplus::draw() {
	
	// draw line
	drawPointTrajectory(values, V3D(0.3, 0.3, 0.3), 1);

	// draw points
	/*
	glColor3d(0, 0.8, 0);
	for(P3D & pt : values) {
		drawSphere(pt, 0.005);
	}
	*/

	// draw Spline
	if(discreteSpline != NULL) {
		drawPointTrajectory(discreteSpline->values, V3D(0.0, 0.0, 0.0), 1);
	}
	glColor3d(1.0, 1.0, 1.0);
	for(P3D & pt : discreteSpline->values) {
		drawSphere(pt, 0.003);
	}
}