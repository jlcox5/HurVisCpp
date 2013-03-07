/*
 *  hurEyePoints.h
 *
 *  Created on: Feb 12, 2013
 *  Jonathan Cox
 *  Clemson University
 *  
 *  This class simply generates random points for the eye of the hurricane that fall within
 *  the radius of the error cone with respect to the appropriate point on the predicted path.
 */

#ifndef HUREYEPOINTS_H_
#define HUREYEPOINTS_H_

#include <vector>
#include "geoFunct.h"
#include "predictedPath.h"

class hurEyePoints{
  private:
     std::vector<Vector2d*> eyePositions;
     Vector3d color;
     double trans;

  public:
     hurEyePoints(predictedPath *);
     void drawPoints();
     void age(double d){ trans = trans - d; }

     double getTrans(){ return trans; }
};

#endif
