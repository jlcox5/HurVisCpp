/* Jonathan Cox
   Clemson University
   Hurricane Visualization Functions
*/

#include "Vector.h"
#include "posPoint.h"
#include "geoFunct.h"
#include "velocityFunct.h"

#ifndef HURVIS_H
#define HURVIS_H


Vector4d nextPoint(Vector4d curPoint, double steps);
double findDeg(double x, double y);
Vector2d closePointOnPath(Vector4d * projPath1, Vector4d * projPath2, Vector4d curPoint);
void buildAdvisory();


#endif
