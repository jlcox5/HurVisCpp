/*
 * predictedPath.cpp
 *
 * Created on: Apr 5, 2011
 * Jonathan Cox
 * Clemson University
 */

#include <fstream>
#include "predictedPath.h"
#include "geoFunct.h"

predictedPath::predictedPath(string filename, std::vector<Vector4d*> * e1, std::vector<Vector4d*> * e2){
   fstream file;
   int curLat, curLon, curBin;
   double lat, lon, bear, speed;
   Vector4d * toPush;

   curLat = 0;
   curLon = 0;
   curBin = 0;

   file.open(filename.c_str(), fstream::in);

   file >> dec >> initBear;
   file >> dec >> initSpeed;

   while(!file.eof() && !file.bad()){
      file >> dec >> lat;
      file >> dec >> lon;
      file >> dec >> bear;
      file >> dec >> speed;
      //cout << "read: " << lat << ", " << lon << ", " << bear << ", " << speed << endl;
      toPush = new Vector4d(lon, lat, bear, speed);
         
      predPathThreeHour.push_back(toPush);
   }

   // For duplicate error caused by improper file reading... need to address
   predPathThreeHour.erase(predPathThreeHour.end()-1);

   file.close();
   buildPathSegments(e1, e2);
}

void predictedPath::buildPathSegments(std::vector<Vector4d*> * e1, std::vector<Vector4d*> * e2){
   std::vector<Vector4d*>::iterator _p0;
   std::vector<Vector4d*>::iterator _p1;
   pathSegment * p;
   double t = 0.0;

   _p1 = predPathThreeHour.begin() + 1;
   for(_p0 = predPathThreeHour.begin(); _p0 != predPathThreeHour.end()-1; _p0++){
      p = new pathSegment((*_p0), (*_p1), t, e1, e2, initBear, initSpeed);
      predPathSeg.push_back(p);
      _p1++;
      t += 3.0;
   }
}
