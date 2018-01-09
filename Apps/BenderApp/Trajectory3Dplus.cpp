

#include <GUILib/GLUtils.h>
#include <FEMSimLib/Node.h>

#include "Trajectory3Dplus.h"



/*
Trajectory3Dplus::~Trajectory3Dplus() {
	delete discreteSpline;
}
*/


void Trajectory3Dplus::createFromNodes(DynamicArray<Node *> const & nodes, dVector const & x)
{
	int n = nodes.size();
	values.resize(n);

	for(int i = 0; i < n; ++i) {
		values[i] = nodes[i]->getCoordinates(x);
	}

	setTValueToLength();
}

V3D Trajectory3Dplus::evaluate_gradient_catmull_rom(double t, bool equalEndpointSlopes)
{
	using T = V3D;

	int size = (int)tValues.size();
	if (t<=tValues[0]) return values[0];
	if (t>=tValues[size-1])	return values[size-1];
	int index = getFirstLargerIndex(t);

	//now that we found the interval, get a value that indicates how far we are along it
	t = (t-tValues[index-1]) / (tValues[index]-tValues[index-1]);

	//approximate the derivatives at the two ends

	T p1 = values[index-1];
	T p2 = values[index];

	T m1 = getSlopeEstimateAtKnot(index-1, equalEndpointSlopes) * (tValues[index]-tValues[index-1]);
	T m2 = getSlopeEstimateAtKnot(index, equalEndpointSlopes) * (tValues[index]-tValues[index-1]);

	double t2;//, t3;
	t2 = t*t;
	//t3 = t2*t;

	// compute the gradient (derivative of the catmull rom interpolation performed in evaluate_catmull_rom() of base class
	// value = p1*(2*t3-3*t2+1) + m1*(t3  -2*t2+t) + p2*(-2*t3+3*t2) + m2*(t3  -  t2);
	T   grad = p1*(6*t2-6*t   ) + m1*(3*t2-4*t +1) + p2*(-6*t2+6*t ) + m2*(3*t2-2*t );
	
	return(grad);

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


void Trajectory3Dplus::setTValueToLength() {
	
	tValues.resize(values.size());

	if(values.size() < 1) {
		return;
	}

	tValues[0] = 0.0;
	for(int i = 1; i < values.size(); ++i) {
		tValues[i] = tValues[i-1] + V3D(values[i-1], values[i]).length();
	}
}


void Trajectory3Dplus::createDiscreteSpline(int n, Trajectory3Dplus & spline)
{
	
	assert(n > 1);

	double t_tot = this->getMaxPosition() - this->getMinPosition();
	double dt = t_tot / static_cast<double>(n-1);

	spline.values.resize(n);
	spline.tValues.resize(n);
	for(int i = 0; i < n; ++i) {
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

	setTValueToLength();
	//updateDiscreteSpline();

}

void Trajectory3Dplus::addKnotBack(P3D const & pt)
{
	values.push_back(pt);

	setTValueToLength();
}


void Trajectory3Dplus::removeKnotInteractive(int knotID)
{
	if(values.size() > 2 && knotID >= 0 && knotID < values.size()) {
		removeKnot(knotID);
		setTValueToLength();
	}
}

int Trajectory3Dplus::getSelectedKnotID(Ray const & ray)
{
	int ID = -1;
	double dis = 2e9;
	for (uint i = 0; i < values.size(); i++) {
		P3D tp = values[i];
		double tDis = ray.getDistanceToPoint(tp) / (sqrt((tp-ray.origin).dot(tp - ray.origin)));
		//Logger::consolePrint("%lf %lf %lf %lf\n", tp.x(), tp.y(), tp.z(), tDis);
		if (tDis < 0.01 && tDis < dis) {
			dis = tDis;
			ID = i;
		}
	}
	return ID;
}


double Trajectory3Dplus::getDistanceToRay(Ray const & ray, P3D *closestPtOnRay)
{
	if(values.size() == 0) {
		closestPtOnRay = NULL;
		return(-1.0);
	}
	else if(values.size() == 1) {
		return(ray.getDistanceToPoint(values[0], closestPtOnRay));
	}
	else {
		double d_min = std::numeric_limits<double>::max();
		double i_min = -1;
		for(int i = 0; i < values.size()-1; ++i) {
			double d;
			d = ray.getDistanceToSegment(values[i], values[i+1], closestPtOnRay);
			if(d < d_min) {
				d_min = d;
				i_min = i;
			}
		}

		return(ray.getDistanceToSegment(values[i_min], values[i_min+1], closestPtOnRay));
	}
}


void Trajectory3Dplus::draw(V3D lineColor, int lineWidth, V3D knotColor, double knotSize) {
	
	// draw line
	if(lineWidth > 0) {
		drawPointTrajectory(values, lineColor, lineWidth);
	}
	// draw points
	if(knotSize > 0.0) {
		glColor3d(knotColor(0), knotColor(1), knotColor(2));
		for(P3D & pt : values) {
			drawSphere(pt, knotSize);
		}	
	}
}



double Trajectory3Dplus::distanceSquaredFromTo(Trajectory3Dplus const & traj1, Trajectory3Dplus const & traj2)
{

	int n = traj1.values.size();
	assert(n > 1);

	double d2 = 0.0;
	double w_sum = 0.0;

	for(int i = 0; i < n; ++i) {
		// find distance
		double d;
		int i_traj2;
		bool within_segment;
		traj2.getMinDistanceLinear(traj1.values[i], d, i_traj2, within_segment);
		// find weight
		double w;
		if(i == 0)        {w = traj1.tValues[1] - traj1.tValues[0];}
		else if(i == n-1) {w = traj1.tValues[n-1] - traj1.tValues[n-2];}
		else {
			w = 0.5*(traj1.tValues[i+1] - traj1.tValues[i-1]);
		}
		// sum
		d2 += d*d * w;
		w_sum += w;
	}

	d2 /= w_sum;

	return(d2);
}