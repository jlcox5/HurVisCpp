/* Jonathan Cox
   Clemson University
   Position Point
*/

#include <math.h>
#include "hurVis.h"
#include "posPoint.h"
#include "simulation.h"

extern simulation * sim;

posPoint::posPoint(int m, int da, double lat, double lon, double deg, double s, int w, int pid){
   month = m;
   day = da;
   latitude = lat;
   longitude = lon;
   degree = deg;
   speed = s;
   wind = w;
   nextPosDif.set(0.0, 0.0);
   pathId = pid;
   
   double velX, velY;
   
   // Determine velocity based on degree and speed
   velX = sin(deg*M_PI/180);
   velY = cos(deg*M_PI/180);
   
   // Multiplied by 2.0 for distance traveled in two hours
   // Speed is in kilometers per hour
   velX = velX*speed*sim->getHours();
   velY = velY*speed*sim->getHours();
   
   // Assign to point
   point.x = lon;
   point.y = lat;
   point.z = velX;
   point.w = velY;
}
