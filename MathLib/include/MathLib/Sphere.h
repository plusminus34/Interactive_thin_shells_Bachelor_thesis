#pragma once

/*
  A sphere
*/

#include "boundingBox.h"

class SimpleSphere{
public:
  SimpleSphere(P3D center_, double radius_) : center(center_), radius(radius_) {}

  SimpleSphere(double x, double y, double z, double radius_) : center(P3D(x,y,z)), radius(radius_) {}

  // does the sphere intersect the bounding box
  bool doesBoundingBoxIntersect(AxisAlignedBoundingBox & box) const;

private:
  P3D center;
  double radius;
};
