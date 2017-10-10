#include "../include/MathLib/Sphere.h"

/*
  A sphere
*/

// does the sphere intersect the bounding box
bool SimpleSphere::doesBoundingBoxIntersect(AxisAlignedBoundingBox & box) const{
  P3D bmin,bmax;
  bmin = box.bmin();
  bmax = box.bmax();

  double d;

  #define COORDINATE_TEST(i)\
  d = bmin[i] - center[i];\
  if (d > 0)\
    dmin += d * d;\
  d = center[i] - bmax[i];\
  if (d > 0)\
    dmin += d * d;

  double dmin = 0;
  COORDINATE_TEST(0)
  COORDINATE_TEST(1)
  COORDINATE_TEST(2)

  #undef COORDINATE_TEST

  return (dmin <= radius * radius);
}
    

