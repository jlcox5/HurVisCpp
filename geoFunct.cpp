/*
 * geoFunct.cpp
 *
 *  Created on: Aug 12, 2010
 *  Jonathan Cox
 *  Clemson University
 */

#include "geoFunct.h"
#include "simulation.h"

extern simulation * sim;
extern int w, h;

// Given lat/long position, return coordinates for the screen
Vector2d translateToScreen(double curLon, double curLat){
   Vector2d toReturn;

   if(sim->getDispNum() == 1){
      w = 1024;
      h = 790;
   }
   else{
      w = 614;
      h = 474;
   }

   toReturn.x = ((sim->getMaxLon()-curLon) / sim->getTotLon())*w;
   toReturn.y = ((sim->getMaxLat()-curLat) / sim->getTotLat())*h;

   if( toReturn.x != toReturn.x || toReturn.y != toReturn.y ){
      cout << "Bad screen point for position: " << curLat << ", " << curLon << endl;
      cout << "maxLon: " << sim->getMaxLon() << "    curLon: " << curLon << "     totalLon: " << sim->getTotLon() << "    w: " << w << endl;
      cout << flush; 
   }

   return toReturn;
}

// Given a latitude, longitude, the velocities in the North/South, and East/West,
// And the angle of travel in degrees, return the next position on the map in
// Lat/Long coordinates
Vector2d locateDestination(double curLon, double curLat, double velX, double velY, double curDeg){
   Vector2d toReturn;

   double pi = 3.14159265;

   if(curDeg == 0){
      curDeg = 360;
   }

   double orgLat, orgLon;
   double dist = sqrt(Sqr(velX) + Sqr(velY));
   double earthRad = 6371.0;

   orgLon = (curLon*pi)/180.0;
   orgLat = (curLat*pi)/180.0;
   dist = dist/earthRad;
   curDeg = (curDeg*pi)/180.0;
   curLat = asin( sin(orgLat) * cos(dist) + cos(orgLat) * sin(dist) * cos(curDeg));
   curLon = orgLon + atan2(sin(curDeg) * sin(dist) * cos(orgLat),
                     cos(dist)-sin(orgLat)*sin(curLat));

   toReturn.x = (180.0*curLon)/M_PI;
   toReturn.y = (180.0*curLat)/M_PI;

   return toReturn;
}

// This function does the same thing as the other, but with the distance already
// computed in km
Vector2d locateDestination_2(double curLon, double curLat, double dist, double curDeg){
   Vector2d toReturn;

   double pi = 3.14159265;

   if(curDeg == 0){
      curDeg = 360;
   }

   double orgLat, orgLon;
   double earthRad = 6371.0;

   orgLon = (curLon*pi)/180.0;
   orgLat = (curLat*pi)/180.0;
   dist = dist/earthRad;
   curDeg = (curDeg*pi)/180.0;
   curLat = asin( sin(orgLat) * cos(dist) + cos(orgLat) * sin(dist) * cos(curDeg));
   curLon = orgLon + atan2(sin(curDeg) * sin(dist) * cos(orgLat),
                     cos(dist)-sin(orgLat)*sin(curLat));

   toReturn.x = (180.0*curLon)/M_PI;
   toReturn.y = (180.0*curLat)/M_PI;

   return toReturn;
}

// Computes the haversine formula to find the distance between two point
double haversine(double lon1, double lon2, double lat1, double lat2){
   double a, b, c, earthRad, dLat, dLon;
   double radLat1, radLat2, radLon1, radLon2;
   earthRad = 6378.1;

   radLat1 = lat1*(M_PI/180.0);
   radLon1 = lon1*(M_PI/180.0);
   radLat2 = lat2*(M_PI/180.0);
   radLon2 = lon2*(M_PI/180.0);

   dLat = (lat2-lat1)*(M_PI/180.0);
   dLon = (lon2-lon1)*(M_PI/180.0);

   a = Sqr(sin(dLat/2.0)) + cos(radLat1)*cos(radLat2) * Sqr(sin(dLon/2.0));
   b = 2.0 * atan2(sqrt(a), sqrt(1.0-a));
   c = earthRad * b;
   return c;
}

// Find the bearing between two points
double findBearing_2(Vector2d * pos1, Vector2d * pos2){
   double dLon, x, y, toReturn;

   dLon = pos2->x - pos1->x;
   y = sin(dLon*(M_PI/180.0))*cos(pos2->y*(M_PI/180.0));
   x = cos(pos1->y*(M_PI/180.0))*sin(pos2->y*(M_PI/180.0)) - 
       sin(pos1->y*(M_PI/180.0))*cos(pos2->y*(M_PI/180.0))*cos(dLon*(M_PI/180.0));
   toReturn = (atan2(y, x)* 180.0) / M_PI;

   if( toReturn < 0.0){
      toReturn = 360.0 + toReturn;
   }

   return toReturn;
}

double crossTrack(Vector4d * projPath1, Vector4d * projPath2, Vector4d curPoint){
   double toReturn, b1, b2, l1;
   Vector2d * curP, * p1, * p2;
   double earthRad = 6378.1;

   p1 = new Vector2d();
   p1->x = projPath1->x;
   p1->y = projPath1->y;

   p2 = new Vector2d();
   p2->x = projPath2->x;
   p2->y = projPath2->y;

   curP = new Vector2d(curPoint.x, curPoint.y);
   //curP->x = curPoint->x;
   //curP->y = curPoint->y;

   b1 = findBearing_2(p1, p2);
   b2 = findBearing_2(p1, curP);

   l1 = haversine(p1->x, curP->x, p1->y, curP->y);

   toReturn = asin(sin(l1/earthRad) * sin(b2-b1))*earthRad;

   return toReturn;
}

std::vector<Vector4d*> smoothCurve(Vector4d * x1, Vector4d * x2, double deg1, double deg2, double ts, int numSeg){
   Vector2d p, p0, p1;
   Vector2d v, v0, v1;
   Vector2d dp, dv;
   Vector2d a, b, c, d;
   Vector4d * push;

   std::vector<Vector2d*>::iterator dataIt;
   std::vector<Vector4d*> toReturn;

   float dt, Bearing, Speed;

   int N = numSeg;       // number of points to interpolate between known points
   float ds = 1.0 / N;
   float seg;
   float lat0, lon0, lat1, lon1, ratio0, ratio1;
   double dydt, dyds0, dyds1;
   double dxdt, dxds0, dxds1;
   double bear0, dist0, bear1, dist1, earthRad;


   dt = ts;             // number of hours between samples
   earthRad = 6371.0;

   // Values for s = 0
   bear0 = DegToRad(deg1);     // Bearing in radians
   dist0 = sqrt(x1->z*x1->z + x1->w*x1->w);       // Speed in km/h
   lat0 = DegToRad(x1->y);   // Latitude in radians
   lon0 = DegToRad(x1->x);   // Longitude in radians
   ratio0 = dist0/earthRad;     // r/R (a radian measure)
   dxdt = ratio0*sin(bear0);    // radians longtitude/h
   dxds0 = dxdt * dt;        // radians per unit of s
   dydt = ratio0*cos(bear0);    // radians latitude/h
   dyds0 = dydt * dt;        // radians per unit of s

   p0 = Vector2d(lon0, lat0);   // position, radians long, lat
   v0 = Vector2d(dxds0, dyds0);    // velocity, radians per unit of s

   // Values for s = 1
   bear1 = DegToRad(deg2);     // Bearing in radians
   dist1 = sqrt(x2->z*x2->z + x2->w*x2->w);         // Speed in km/h
   lat1 = DegToRad(x2->y);   // Latitude in radians
   lon1 = DegToRad(x2->x);   // Longitude in radians
   ratio1 = dist1/earthRad;     // r/R (a radian measure)
   dxdt = ratio1*sin(bear1);    // radians longtitude/h
   dxds1 = dxdt * dt;        // radians per unit of s
   dydt = ratio1*cos(bear1);    // radians latitude/h
   dyds1 = dydt * dt;        // radians per unit of s

   p1 = Vector2d(lon1, lat1);   // position, radians long, lat
   v1 = Vector2d(dxds1, dyds1);    // velocity, radians per unit of s

   // Solve for the cubic coefficients
   dp = p1 - p0;          // delta p between points (radians long and lat)
   dv = v1 - v0;          // delta v between points (radians per unit of s)

   a = p0;
   b = v0;
   c = 3 * dp - 2*v0 - v1;
   d = -2 * (dp - v0) + dv;

   // Now that coeffecients are known, interpolate points on the curve


   //cout << "start point: " << s->getLat() << "," << s->getLon() << " " << s->getSpeed() << " " << s->getDeg() << "\n";
   for(int i = 1; i < N; i++){
     seg = i * ds;    // compute parameter s at this subpoint on the curve
     //cout << "   Seg: " << seg << "\n";

     // Find longitude and latitude in degrees
     p = RadToDeg(a + (b + (c + d * seg) * seg) * seg);
     // Find the longitudinal velocity at this point in radians/hr
     v = (v0 + (2 * c + 3 * d * seg) * seg) / dt;

     Speed = earthRad * v.norm();          // speed in km/h
     Bearing = RadToDeg(atan2(v.x, v.y));     // bearing in degrees
     if(Bearing < 0){
        Bearing = 360 + Bearing;
     }

     push = new Vector4d(p.x, p.y, Bearing, Speed);
     //np = new posPoint(m, da, p.y, p.x, Bearing, Speed, f->getWind(), pid);
     //cout << "    new point: " << np->getLon() << "," << np->getLat() << "\n";
     //cout << "       with speed: " << Speed << "    bearing: " << Bearing << "\n";
     toReturn.push_back(push);
   }

   // Find value for the last generated point and final position
   return toReturn;
   //cout << "finish point: " << f->getLat() << "," << f->getLon() << " " << f->getSpeed() << " " << f->getDeg() << "\n";

}

// Returns the smallest degree difference between two angles
double findDif(double a, double b){
   double toRet;

   toRet = a - b;
   if(toRet > 180){
      toRet = toRet - 360;
   }
   if(toRet < -180){
      toRet = toRet + 360;
   }
   return toRet;
}
