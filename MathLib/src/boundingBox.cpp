//  Bounding Box

#include <float.h>
#include <vector>
#include <iostream>
using namespace std;

#include "../include/MathLib/boundingBox.h"
//#include <GUILib/GLUtils.h>

void AxisAlignedBoundingBox::addPoint(const P3D& p) {
	for (int i = 0; i < 3; i++) {
		if (p[i] < bmin_[i]) bmin_[i] = p[i];
		if (p[i] > bmax_[i]) bmax_[i] = p[i];
	}
	updateData();
}

//sets this box to an empty state...
void AxisAlignedBoundingBox::empty() {
	bmin_[0] = bmin_[1] = bmin_[2] = DBL_MAX;
	bmax_[0] = bmax_[1] = bmax_[2] = -DBL_MAX;
	updateData();
}

void AxisAlignedBoundingBox::regularize() {
  P3D center_ = (bmin_ + bmax_) * 0.5;
  P3D halfside_ = (bmax_ + bmin_*-1) * 0.5;

  double maxHalf = halfside_[0];
  if (halfside_[1] > maxHalf)
    maxHalf = halfside_[1];
  if (halfside_[2] > maxHalf)
    maxHalf = halfside_[2];

  P3D cubeHalf_ = P3D(maxHalf,maxHalf,maxHalf);

  bmin_ = P3D(center_ - cubeHalf_);
  bmax_ = center_ + cubeHalf_;

  updateData();
}

void AxisAlignedBoundingBox::updateData(){
	P3D min, max;
	min.x() = MIN(bmin_.x(), bmax_.x());
	min.y() = MIN(bmin_.y(), bmax_.y());
	min.z() = MIN(bmin_.z(), bmax_.z());
	max.x() = MAX(bmin_.x(), bmax_.x());
	max.y() = MAX(bmin_.y(), bmax_.y());
	max.z() = MAX(bmin_.z(), bmax_.z());

	bmin_ = min;
	bmax_ = max;

	center_ = (bmin_ + bmax_) * 0.5;
	halfSides_ = (bmax_ + bmin_ * -1) * 0.5;
}

void AxisAlignedBoundingBox::verifyBox(){
  if (!((bmin_[0] <= bmax_[0]) && (bmin_[1] <= bmax_[1]) && (bmin_[2] <= bmax_[2])))
    printf("Error: inconsistent bounding box.\n");
}

// TODO: move to suitable location
//void AxisAlignedBoundingBox::render(){
//  // render the bounding box
//  P3D p0(bmin_[0],bmin_[1],bmin_[2]);
//  P3D p1(bmax_[0],bmin_[1],bmin_[2]);
//  P3D p2(bmax_[0],bmax_[1],bmin_[2]);
//  P3D p3(bmin_[0],bmax_[1],bmin_[2]);
//
//  P3D p4(bmin_[0],bmin_[1],bmax_[2]);
//  P3D p5(bmax_[0],bmin_[1],bmax_[2]);
//  P3D p6(bmax_[0],bmax_[1],bmax_[2]);
//  P3D p7(bmin_[0],bmax_[1],bmax_[2]);
//
//  #define VTX(i) (i)[0],(i)[1],(i)[2]
//  glBegin(GL_LINES);
//    glVertex3d(VTX(p0));
//    glVertex3d(VTX(p1));
//    glVertex3d(VTX(p1));
//    glVertex3d(VTX(p2));
//    glVertex3d(VTX(p2));
//    glVertex3d(VTX(p3));
//    glVertex3d(VTX(p3));
//    glVertex3d(VTX(p0));
//
//    glVertex3d(VTX(p0));
//    glVertex3d(VTX(p4));
//    glVertex3d(VTX(p1));
//    glVertex3d(VTX(p5));
//    glVertex3d(VTX(p2));
//    glVertex3d(VTX(p6));
//    glVertex3d(VTX(p3));
//    glVertex3d(VTX(p7));
//
//    glVertex3d(VTX(p4));
//    glVertex3d(VTX(p5));
//    glVertex3d(VTX(p5));
//    glVertex3d(VTX(p6));
//    glVertex3d(VTX(p6));
//    glVertex3d(VTX(p7));
//    glVertex3d(VTX(p7));
//    glVertex3d(VTX(p4));
//  glEnd();
//
//  #undef VTX
//}

// should this be turned into a self-modifying function?
void AxisAlignedBoundingBox::expand(double expansionFactor){
  bmin_ = center_ + halfSides_ * -expansionFactor;
  bmax_ = center_ + halfSides_ * expansionFactor;

  updateData();
}

bool AxisAlignedBoundingBox::lineSegmentIntersection(P3D segmentStart, P3D segmentEnd, P3D * intersection){
  #define NUMDIM  3
  #define RIGHT   0
  #define LEFT    1
  #define MIDDLE  2

  V3D dir = segmentEnd - segmentStart;
  bool inside = true;
  char quadrant[NUMDIM];
  register int i;
  int whichPlane;
  double maxT[NUMDIM];
  double candidatePlane[NUMDIM];

  /* Find candidate planes; this loop can be avoided if
  rays cast all from the eye(assume perpsective view) */
  for (i=0; i<NUMDIM; i++)
    if(segmentStart[i] < bmin_[i]) 
    {
      quadrant[i] = LEFT;
      candidatePlane[i] = bmin_[i];
      inside = false;
    }
    else if (segmentStart[i] > bmax_[i]) 
    {
      quadrant[i] = RIGHT;
      candidatePlane[i] = bmax_[i];
      inside = false;
    }
    else   
    {
      quadrant[i] = MIDDLE;
    }
                                                                                                                                                             
  /* Ray origin inside bounding box */
  if(inside)      
  {
    *intersection = segmentStart;
    return (true);
  }

  /* Calculate T distances to candidate planes */
  for (i = 0; i < NUMDIM; i++)
    if (quadrant[i] != MIDDLE && dir[i] !=0.)
      maxT[i] = (candidatePlane[i]-segmentStart[i]) / dir[i];
    else
      maxT[i] = -1.;

  /* Get largest of the maxT's for final choice of intersection */
  whichPlane = 0;
  for (i = 1; i < NUMDIM; i++)
    if (maxT[whichPlane] < maxT[i])
      whichPlane = i;

  /* Check final candidate actually inside box */
  if (maxT[whichPlane] < 0.0) 
    return (false);
  if (maxT[whichPlane] > 1.0) 
    return (false); // remove this for ray

  for (i = 0; i < NUMDIM; i++)
  {
    if (whichPlane != i) 
    {
      (*intersection)[i] = segmentStart[i] + maxT[whichPlane] *dir[i];
      if ((*intersection)[i] < bmin_[i] || (*intersection)[i] > bmax_[i])
        return (false);
    } 
    else 
    {
      (*intersection)[i] = candidatePlane[i];
    }
  }

  return (true);                          /* ray hits box */

  #undef NUMDIM
  #undef RIGHT
  #undef LEFT
  #undef MIDDLE
}

double AxisAlignedBoundingBox::distanceToPoint(P3D point){
  double distance = 0.0;
  for(int dim=0; dim<3; dim++)
  {
    if (point[dim] < bmin_[dim])
      distance += (bmin_[dim] - point[dim]) * (bmin_[dim] - point[dim]);
    if (point[dim] > bmax_[dim])
      distance += (point[dim] - bmax_[dim]) * (point[dim] - bmax_[dim]);
  }
  return sqrt(distance);
}

bool AxisAlignedBoundingBox::isInside(P3D point)
{
	return point[0] >= bmin_[0] && point[0] <= bmax_[0]
		&& point[1] >= bmin_[1] && point[1] <= bmax_[1]
		&& point[2] >= bmin_[2] && point[2] <= bmax_[2];
}

bool AxisAlignedBoundingBox::getDistanceToRayOriginIfHit(const Ray& ray, double* distToOrigin)
{
	double t0 = 0;
	double t1 = std::numeric_limits<double>::infinity();

	for (int i = 0; i < 3; i++) {
		if (ray.direction[i] != 0.0) {
			double tx1 = (bmin_[i] - ray.origin[i]) / ray.direction[i];
			double tx2 = (bmax_[i] - ray.origin[i]) / ray.direction[i];

			t0 = max(t0, min(tx1, tx2));
			t1 = min(t1, max(tx1, tx2));
		}
	}

	if (distToOrigin)
		*distToOrigin = t0;

	return t0 <= t1;
}

void AxisAlignedBoundingBox::print()
{
  cout << bmin_ << " " << bmax_ << endl;
}

P3D AxisAlignedBoundingBox::getVertex(int index)
{
	switch (index)
	{
	case 0: return P3D(bmin_[0], bmin_[1], bmin_[2]);
	case 1: return P3D(bmax_[0], bmin_[1], bmin_[2]);
	case 2: return P3D(bmin_[0], bmax_[1], bmin_[2]);
	case 3: return P3D(bmax_[0], bmax_[1], bmin_[2]);
	case 4: return P3D(bmin_[0], bmin_[1], bmax_[2]);
	case 5: return P3D(bmax_[0], bmin_[1], bmax_[2]);
	case 6: return P3D(bmin_[0], bmax_[1], bmax_[2]);
	case 7: return P3D(bmax_[0], bmax_[1], bmax_[2]);
	default:
		return P3D();
	}

	return P3D();
}