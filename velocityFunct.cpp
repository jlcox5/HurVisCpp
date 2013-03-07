/*
 * velocityFunct.cpp
 *
 *  Created on: Aug 12, 2010
 *  Jonathan Cox
 *  Clemson University
 */

#include <math.h>
#include <limits.h>
#include "hurVis.h"
#include "posPoint.h"
#include "path.h"
#include "Matrix.h"
#include "advisory.h"

// Given two positions and a time, figure out the velocity components needed to travel
// the distance between the two in the given time frame

Vector2d * genVelocity(double lon1, double lon2, double lat1, double lat2, double time){
   Vector2d * toReturn, * temp1, * temp2;
   double bearing, speed;

   toReturn = new Vector2d;
   temp1 = new Vector2d(lon1, lat1);
   temp2 = new Vector2d(lon2, lat2);

   // Speed is distance over time
   speed =  haversine(lon1, lon2, lat1, lat2)/time;

   bearing = findBearing_2(temp1, temp2);

   toReturn->x = sin(bearing*M_PI/180.0)*speed;
   toReturn->y = cos(bearing*M_PI/180.0)*speed;

   return toReturn;
}

// Normalize the velocity components and return the result
Vector2d normVel(double x, double y){
   Vector2d toReturn;

   toReturn.x = x/( sqrt(Sqr(x)+Sqr(y)));
   toReturn.y = y/( sqrt(Sqr(x)+Sqr(y)));

   return toReturn;
}

