/* Jonathan Cox
   Clemson University
   Hurricane Visualization Functions
*/

#include <math.h>
#include <limits.h>
#include "hurVis.h"
#include "posPoint.h"
#include "path.h"
#include "Matrix.h"
#include "advisory.h"
#include "simulation.h"
#include "gridStruct.h"

//extern int wWidth, wHeight;
//int w = 1024, h = 790;
int w = 768, h = 593;

extern simulation * sim;
extern gridStruct * dataGrid;
// Defined in path.cpp
extern int totalDistance;

extern path * testPath;
extern path * curPath;

extern int interactive, hitButton;
extern double projPathDist;

extern int chooseForward;

extern double curDistance;

extern int preOnly;

extern double speedRatio;
extern double bearRatio;

double weight = 0.0;

// For random number generation
unsigned int counter = time(NULL);

// KN search parameter and distance threshold in km
int searchPop = 300;

extern int totalPaths;

double bMod = 0.25;//0.75;
double sMod = 1; //15;//0.5;//0.5;//1.0;
double e = 1;
double d = 1;

//extern double stdDevAlpha;

// How to do the velocity
int velType = 1;

// Error rad - year dependant.  These are good for 2010
// Values are in km

double errorRadR[] = {0.0, 50.004, 100.471, 144.302, 187.515, 283.263, 390.463};
double errorRadius[] = {0.0, 50.004, 100.471, 144.302, 187.515, 283.263, 390.463};
//double errorRadius[] = {0.0, 66.672, 114.824, 157.42, 200.016, 298.172};
//double errorRad[] = {66.672, 66.672, 66.672, 66.672, 66.672, 66.672};

extern std::vector<Vector4d*> projPath;
extern std::vector<Vector4d*>::iterator projPathIt;

int debugPrint = 0;

// Generates the next point for a path list
// Returns the long, lat, speed in kph, and bearing 
// in the x(north/south) and y(east/west) directions
Vector4d nextPoint(Vector4d curPoint, double steps){
   Vector4d toReturn;
   Vector4d toSearch, found;
   
   std::vector<double> rList;
   std::vector<posPoint*>::iterator _p;
   
   int r;
   double chosen, ch1, ch2;
   
   Vector4d newPos;
   Vector2d combVel;
   Vector2d curVelNorm;
   Vector2d comVelNorm;
   Vector2d avgVel, tempVel, perpVel, distVel, adjustedVel;
   Vector2d curToN, tempVec, tDir;

   int cDeg;
   int degLat, degLon, bin;
   double bd, sd;
   double newBear, fSpeed;

   double min;
   int speedHistPre;
   int bearHistPre;
   min = steps*sim->getHours()*60;

   if(steps == 0){
      //cout << "Step: " << steps << endl;
      debugPrint = 0;
   }

   //sim->nList.clear();
   // New method using bins //
   // Generate random number
   srand ( counter );
   counter = (counter*21)%UINT_MAX;

   r = rand() % 10000000+1;
   chosen = (double)r/10000000.0;

   double chosen2;
   srand ( counter );
   counter = (counter*21)%UINT_MAX;

   r = rand() % 10000000+1;
   chosen2 = (double)r/10000000.0;

   //cout << "lat: " << curPoint.y << " lon: " << curPoint.x << "\n";
   degLat = (int)(abs(floor(abs(curPoint.y)))-abs(dataGrid->getMinLat()));
   degLon = (int)(abs(floor(abs(curPoint.x)))-abs(dataGrid->getMinLon()));
   newBear = curPoint.w;
   cDeg = (int)curPoint.w;
   bin = (int)floor( cDeg/dataGrid->getDegPerBin() );
   if( bin == 6 ){
      bin -= 1;
   }

   //cout << "datLat: " << degLat << " degLon: " << degLon << " bin: " << bin << "\n";
   //cout << "curPoint: " << curPoint << endl;
   //cout << flush;
   bd = 0.0;
   if(degLat < dataGrid->getDegLat() && degLat > -1 && degLon < dataGrid->getDegLon() && degLon > -1){
      if(dataGrid->grid[degLat]->at(degLon)->at(bin)->b.size() != 0){
        
         // Generate speed difference from probability density estimator
         speedHistPre = 0;
         srand ( counter );
         counter = (counter*21)%UINT_MAX;

         r = rand() % 10000000+1;
         ch1 = (double)r/10000000.0;

         if(ch1 < speedRatio){ speedHistPre = 1; }

         // For new vis... need to remove for old method
         //speedHistPre = 0;
         sd = dataGrid->grid[degLat]->at(degLon)->at(bin)->getProbSpeed3Hour(chosen2, min, speedHistPre);
         fSpeed = sd; 

         // bd is multiplied by the hours per time step... currently this is a value in simulation
         // It needs to be linked here instead of hardcoded
         bearHistPre = 0; 
         srand ( counter );
         counter = (counter*21)%UINT_MAX;

         r = rand() % 10000000+1;
         ch2 = (double)r/10000000.0;

         if(ch2 < bearRatio){ bearHistPre = 1; }

         // For new vis... need to remove for old method
         //bearHistPre = 0;

         bd = dataGrid->grid[degLat]->at(degLon)->at(bin)->getProbBear3Hour(chosen, min, bearHistPre);

         //cout << "Resorting to new: " << found.z << ", " << found.w << "   -   " << comLat << ", " << comLon << "\n";
      }
      else{
         fSpeed = 0.0;
         newBear = 0.0;
      }
   }
   else{
      fSpeed = 0.0;
      newBear = 0.0;
   }

   // Combine changes
   toReturn.z = sim->getAlpha()*fSpeed + curPoint.z;
   if(toReturn.z < 0.0){
      toReturn.z = 0.0;
   }
   toReturn.w = fmod(sim->getAlpha()*bd + curPoint.w, 360.0);
   if(toReturn.w < 0){
      toReturn.w += 360.0;
   }

   // Find the new destination based on the current lat, lon, the current
   // velocity components (used to compute the distance), and the bearing
   newPos = locateDestination_2(curPoint.x, curPoint.y, toReturn.z*3.0, toReturn.w);
   if(newPos.x != newPos.x && newPos.y != newPos.y){
      cout << "Found nan with: " << curPoint << " Speed " << toReturn.z << " avgDeg: " << toReturn.w << "\n";
   }

   // Return the new position
   toReturn.x = newPos.x;
   toReturn.y = newPos.y;

   if(toReturn.x != toReturn.x && toReturn.y != toReturn.y){
      cout << "NAN FOUND: " << toReturn << "\n";
   }
   return toReturn;
}

double findDeg(double x, double y){
   double toReturn;
   Vector2d velN;

   velN = normVel(x, y);
   if(velN.x != velN.x || velN.y != velN.y){
      velN.x = 0;
      velN.y = 0;
   }

   toReturn = atan2(x, y);
   toReturn = (toReturn * 180.0) / M_PI;

   if( toReturn < 0.0){
      toReturn = 360.0 + toReturn;
   }

   return toReturn;
}

Vector2d closePointOnPath(Vector4d * projPath1, Vector4d * projPath2, Vector4d curPoint){
   Vector2d toReturn;
   double l1, l3, lx;

   l1 = haversine(projPath1->x, curPoint.x, projPath1->y, curPoint.y);
   l3 = crossTrack(projPath1, projPath2, curPoint);

   lx = sqrt(Sqr(l1)-Sqr(l3));

   toReturn = locateDestination_2(projPath1->x, projPath1->y, lx, projPath1->z);
   return toReturn;
}

void buildAdvisory(){
   Vector2d * toPush;
   std::vector<Vector2d*> projPathPos;
   advisory * newAdv;

   // Katrina 27AUG05 1500
   // Add projected path points
   toPush = new Vector2d(-85.0, 24.5);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-86.0, 24.6);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-87.6, 25.3);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-89.0, 26.7);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-89.9, 28.6);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-89.5, 33.0);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-86.0, 37.5);
   projPathPos.push_back(toPush);

   newAdv = new advisory(projPathPos, 24.5, -85.0, 275.0, 11.11, string("kat3Hr.txt"));
   sim->advList.push_back(newAdv);

   projPathPos.clear();

   // Gustav 31AUG08 0700
   toPush = new Vector2d(-85.0, 24.2);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-86.3, 25.6);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-88.4, 27.5);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-90.4, 29.1);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-92.2, 30.5);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-93.7, 31.7);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-94.5, 32.0);
   projPathPos.push_back(toPush);

   newAdv = new advisory(projPathPos, 24.2, -85.0, 320.0, 16.11, string("gustav3Hr.txt"));
   sim->advList.push_back(newAdv);

   projPathPos.clear();

   // Alex 29JAn10 0300
   toPush = new Vector2d(-91.6, 21.0);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-92.3, 22.2);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-93.5, 23.6);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-95.2, 24.5);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-96.9, 25.2);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-99.6, 25.7);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-102.0, 27.0);
   projPathPos.push_back(toPush);

   newAdv = new advisory(projPathPos, 21.0, -91.6, 0.0, 7.408, string("alex3Hr.txt"));
   sim->advList.push_back(newAdv);

   projPathPos.clear();

   // Ivan 14SEP04 0300
   toPush = new Vector2d(-85.4, 22.0);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-86.2, 23.1);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-87.3, 24.8);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-88.2, 26.6);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-88.5, 28.5);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-87.5, 32.0);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-86.0, 34.0);
   projPathPos.push_back(toPush);

   newAdv = new advisory(projPathPos, 22.0, -85.4, 325.0, 9.206, string("ivan3Hr2.txt"));
   sim->advList.push_back(newAdv);

   projPathPos.clear();

   /*// Wilma - 22OCT05 0900
   // Add projected path points
   toPush = new Vector2d(-87.2, 20.9);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-87.3, 21.5);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-87.0, 22.0);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-86.0, 23.0);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-83.5, 24.5);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-74.0, 32.0);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-64.0, 41.0);
   projPathPos.push_back(toPush);

   newAdv = new advisory(projPathPos, 20.9, -87.2, 0.0, 0.0, "wilma3Hr.txt");
   sim->advList.push_back(newAdv);

   projPathPos.clear();

   // Emily - 17JUL05 1500
   // Add projected path points
   toPush = new Vector2d(-83.6, 18.6);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-85.9, 19.5);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-88.9, 20.9);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-91.6, 22.2);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-94.2, 23.1);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-99.0, 24.5);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-104.0, 24.5);
   projPathPos.push_back(toPush);

   newAdv = new advisory(projPathPos, 18.6, -83.6, 290.0, 31.48, "emily3Hr.txt");
   sim->advList.push_back(newAdv);*/

   projPathPos.clear();

   // Rita - 21SEP05 2100
   // Add projected path points
   toPush = new Vector2d(-86.8, 24.4);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-88.5, 24.5);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-90.6, 25.2);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-92.7, 26.0);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-94.5, 27.0);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-97.0, 30.5);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-97.0, 33.0);
   projPathPos.push_back(toPush);

   newAdv = new advisory(projPathPos, 24.4, -86.8, 275.0, 20.37, string("rita3Hr.txt"));
   sim->advList.push_back(newAdv);

   projPathPos.clear();

   /*// Ivan - 13SEP04 0900
   // Add projected path points
   toPush = new Vector2d(-83.9, 20.2);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-84.8, 21.2);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-86.0, 22.5);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-87.1, 24.2);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-87.5, 26.0);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-87.5, 30.0);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-85.5, 33.5);
   projPathPos.push_back(toPush);

   newAdv = new advisory(projPathPos, 20.2, -83.9, 300.0, 14.8, "ivan3Hr.txt");
   sim->advList.push_back(newAdv);*/

   projPathPos.clear();

   // Ida - 08NOV09 2100
   // Add projected path points
   toPush = new Vector2d(-86.3, 22.2);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-87.4, 23.9);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-88.0, 26.7);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-87.9, 29.3);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-86.7, 30.7);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-83.0, 31.0);
   projPathPos.push_back(toPush);
   toPush = new Vector2d(-79.5, 30.0);
   projPathPos.push_back(toPush);

   newAdv = new advisory(projPathPos, 22.2, -86.3, 330.0, 16.67, string("ida3Hr.txt"));
   sim->advList.push_back(newAdv);
}
