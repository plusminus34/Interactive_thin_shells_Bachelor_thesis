#pragma once

//  Bounding Box
#include <vector>
#include "Ray.h"
#include "P3D.h"
#include "V3D.h"

class AxisAlignedBoundingBox{
public:
  AxisAlignedBoundingBox(P3D bmin_g=P3D(0,0,0), P3D bmax_g=P3D(1,1,1)): bmin_(bmin_g), bmax_(bmax_g) { 
	  P3D min, max;
	  min.x() = MIN(bmin_.x(), bmax_.x());
	  min.y() = MIN(bmin_.y(), bmax_.y());
	  min.z() = MIN(bmin_.z(), bmax_.z());
	  max.x() = MAX(bmin_.x(), bmax_.x());
	  max.y() = MAX(bmin_.y(), bmax_.y());
	  max.z() = MAX(bmin_.z(), bmax_.z());

	  bmin_ = min;
	  bmax_ = max;
	  updateData();
  }
 
  // accessors
  P3D bmin() { return bmin_;}
  P3D bmax() { return bmax_;}

  P3D center() { return center_;}
  P3D halfSides() { return halfSides_;}
  double diameter() { return V3D(bmin_, bmax_).length(); }

  // mutators
  void setbmin(P3D bmin_g) { bmin_=bmin_g; updateData();}
  void setbmin(double x, double y, double z) { bmin_=P3D(x,y,z); updateData();}
  void setbmax(P3D bmax_g) { bmax_=bmax_g; updateData();}
  void setbmax(double x, double y, double z) { bmax_=P3D(x,y,z); updateData();}

  // incrementally adjust the size of the bounding box by adding new points to it...
  void addPoint(const P3D& p);
  //sets this box to an empty state...
  void empty();

  // TODO: put this somewhere else
  //void render();

  double distanceToPoint(P3D point);
  bool isInside(P3D point);

  // sanity check bmin <= bmax
  void verifyBox();

  // x,y,z can only be 0 or 1.
  P3D getVertex(int index);

  // expands from the center 
  // factor of 1.0 indicates no expansion
  void expand(double expansionFactor);
  void regularize(); // converts the box into one with all sides equal

  bool lineSegmentIntersection(P3D segmentStart, P3D segmentEnd, P3D * intersection);
  bool getDistanceToRayOriginIfHit(const Ray& ray, double* distToOrigin = NULL);

  void print();

 

protected:

  void updateData(); // updates center and half-sides
  P3D center_, halfSides_;
  P3D bmin_,bmax_;
};

