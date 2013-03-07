/*
 * predictedPath.h
 *
 * Creadted on: Apr 5, 2011
 * Jonathan Cox
 * Clemosn University
 */

#include <iostream>
#include <vector>
#include "Vector.h"
#include "pathSegment.h"

#ifndef PREDICTEDPATH_H_
#define PREDICTEDPATH_H_

class predictedPath{
   protected:
      Vector2d midPoint;
      double initBear, initSpeed;

   public:
      std::vector<Vector4d*> predPathThreeHour;
      std::vector<pathSegment*> predPathSeg;

      predictedPath(char * filename, std::vector<Vector4d*> * e1, std::vector<Vector4d*> * e2);
      void buildPathSegments(std::vector<Vector4d*> * e1, std::vector<Vector4d*> * e2);
};

#endif
