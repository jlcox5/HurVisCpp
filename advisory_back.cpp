/*
 * advisory.cpp
 *
 *  Created on: Aug 20, 2010
 *  Jonathan Cox
 *  Clemson University
 */

#ifdef __APPLE__
#  include <GLUT/glut.h>
#else
#  include <GL/glut.h>
#  include <GL/glu.h>
#endif

#include <iostream>
#include <fstream>
#include <algorithm>
#include "advisory.h"
#include "simulation.h"

extern simulation * sim;

extern double errorRadR[];

extern int averageOnAll;

Vector4d * adv;
Vector4d * adv2;

int sortInd = 0;

bool sp(path * p1, path * p2){
   double a, b;
   double bDif_1 = 1.0;
   double bDif_2 = 1.0;

   // Use determinate to find side of line
   bDif_1 = (adv2->x - adv->x)*(p1->latLonList[sortInd]->y - adv->y) - 
            (adv2->y - adv->y)*(p1->latLonList[sortInd]->x - adv->x);
   bDif_2 = (adv2->x - adv->x)*(p2->latLonList[sortInd]->y - adv->y) - 
            (adv2->y - adv->y)*(p2->latLonList[sortInd]->x - adv->x);

   bDif_1 <= 0 ? bDif_1 = 1.0 : bDif_1 = -1.0;
   bDif_2 <= 0 ? bDif_2 = 1.0 : bDif_2 = -1.0;

   bDif_1 <= 0 ? p1->setSide(1) : p1->setSide(-1);
   bDif_2 <= 0 ? p2->setSide(1) : p2->setSide(-1);

   if(sortInd < 22){
      a = bDif_1*haversine(p1->latLonList[sortInd]->x, adv->x, p1->latLonList[sortInd]->y, adv->y);
      b = bDif_2*haversine(p2->latLonList[sortInd]->x, adv->x, p2->latLonList[sortInd]->y, adv->y);
   }
   else{
      a = bDif_1*haversine(p1->latLonList[sortInd]->x, adv2->x, p1->latLonList[sortInd]->y, adv2->y);
      b = bDif_2*haversine(p2->latLonList[sortInd]->x, adv2->x, p2->latLonList[sortInd]->y, adv2->y);
   }

   //delete advVec;
   return abs(a) < abs(b);
}

advisory::advisory(std::vector<Vector2d*> projPathPos,
      double curLat, double curLon, double curDeg, double curSpeed, char * predFile){

   chip * c;

   // Assign position, etc
   lat = curLat;
   lon = curLon;
   deg = curDeg;
   presLat = -1;
   presLon = -1;
   // No longer using initial speed.  Using speed based on projected path
   //speed = curSpeed;

   // Build the projected path
   buildProjPath(projPathPos, curDeg);
   buildErrorCone();
   //speed = (haversine(projPathFull[0]->x, projPathFull[1]->x, projPathFull[0]->y, projPathFull[1]->y))/9;
   speed = curSpeed;
   //pre = new predictedPath(predFile, &errorCone1, &errorCone2);
   pre = new predictedPath(predFile, &ecSmooth1, &ecSmooth2);
   adv = projPath[4];
   adv2 = projPath[5];

   // Build chip stack
   int px, py;
   int i, j, chipNum, chipVal;
   double radVal, pRadVal;
   std::vector<int> * sectList;
   px = py = 45;
   chipNum = 14;
   chipVal = 0;
   radVal = 0;
   pRadVal = 4.5 * 7;

   for(i = 0; i < chipNum; i++){

      if(i < 2){ chipVal = 20; radVal = 4.5; }
      else if(i < 6){ chipVal = 10; pRadVal = radVal; radVal = 3.16;}
      else if(i < 9){ chipVal = 5; pRadVal = radVal; radVal = 2.23; }
      else{ chipVal = 1; pRadVal = radVal; radVal = 1; }

      // Set positions - x
      if(i == 0 || i == 2 || i == 6 || i == 9){
         px = 823 + radVal*7;
      }
      else{
         px += (radVal * 7) * 2 + 4;
      }

      // Set Positions - y
      if(i == 2 || i == 6 || i == 9){
         py += (pRadVal*7) + (radVal * 7) + 4;
      }

      c = new chip(chipVal, px, py, radVal);
      chips.push_back(c);
   }

   // Set sector positions
   for(i = 0; i < 8; i++){
      sectList = new std::vector<int>();
      for(j = 0; j < chipNum; j++){
         sectList->push_back(0); 
      }
      sectorFull.push_back(sectList);
   }

   rad = 250;
   buildTargetArea();
   buildSectors();
}

void advisory::buildProjPath(std::vector<Vector2d*> projPathPos, double projPathDir){

   Vector4d * toAdd;
   Vector2d * segAdd;
   Vector2d temp;
   Vector2d prev;
   int seg = 0;
   double time = 0.0, tempDist = 0.0;

   std::vector<Vector2d*>::iterator p1;
   std::vector<Vector2d*>::iterator p2;
   std::vector<Vector4d*>::iterator projPathIt;

   p1 = projPathPos.begin();
   p2 = projPathPos.begin()+1;

   for(p1 = projPathPos.begin(); p2 != projPathPos.end(); p1++){

      // Figure out the duration of the line segment
      if(seg == 0){
         time = 9.0;
      }
      else if(seg == 1 || seg == 2 || seg == 3){
         time = 12.0;
      }
      else if(seg == 4){
         time = 24.0;
      }
      else{
         time = 24.0;
      }

      segAdd = genVelocity((*p1)->x, (*p2)->x, (*p1)->y, (*p2)->y, time);
      segmentVelocity.push_back(segAdd);

      toAdd = new Vector4d;
      toAdd->x = (*p1)->x;
      toAdd->y = (*p1)->y;

      if(p1 == projPathPos.begin()){
         toAdd->z = projPathDir;
      }
      else{
         toAdd->z = findBearing_2((*p1), (*p2));
      }

      tempDist = haversine((*p1)->x, (*p2)->x, (*p1)->y, (*p2)->y);
      toAdd->w = tempDist/time*sim->getHours();
      projPathIt = projPath.end();
      projPath.insert(projPathIt, toAdd);

      projPathDist += tempDist;

      p2++;
      seg++;

      if(p2 == projPathPos.end()){
         p1++;
         toAdd = new Vector4d((*p1)->x, (*p1)->y, toAdd->z, toAdd->w);
         projPath.push_back(toAdd);
      }
   }
   //cout << "projPath size: " << projPath.size() << endl;
   buildProjPathFull();
}

void advisory::buildProjPathFull(){
   Vector4d * projPath1, * projPath2;
   std::vector<Vector4d*>::iterator projPathIt;
   std::vector<Vector2d*>::iterator _f;

   double tempError, t, subt, setNHS, f, subt2, setNHS2, f2;
   int in1, in2, pathSeg;
   Vector2d * intPos;

   int steps = 0;

   projPathFull.clear();
   distWeightCone.clear();

   while(steps <= 69/sim->getHours()){
      t = steps*sim->getHours();
      // Find i, subt and setNHS based on the hours completed so far
      if(t <= 9){
         in1 = 0;
         in2 = 1;
         subt = 0;
         setNHS = 9;
         pathSeg = 0;
         subt2 = 0;
         setNHS2 = 9;
      }
      else if(t <= 21){
         in1 = 1;
         in2 = 2;
         subt = 9;
         setNHS = 12;
         pathSeg = 1;
         subt2 = 9;
         setNHS2 = 12;
      }
      else if(t <= 33){
         in1 = 2;
         in2 = 3;
         subt = 21;
         setNHS = 12;
         pathSeg = 2;
         subt2 = 21;
         setNHS2 = 12;
      }
      else if(t <= 45){
         in1 = 3;
         in2 = 4;
         subt = 33;
         setNHS = 12;
         pathSeg = 3;
         subt2 = 33;
         setNHS2 = 12;
      }
      else if(t <= 57){
         in1 = 4;
         in2 = 5;
         subt = 45;
         setNHS = 12;
         pathSeg = 4;
         subt2 = 45;
         setNHS2 = 24;
      }
      else{
         in1 = 5;
         in2 = 6;
         subt = 57;
         setNHS = 12;
         pathSeg = 4;
         subt2 = 45;
         setNHS2 = 24;
      }

      // Find the weight value
      f = (t - subt)/setNHS;
      f2 = (t - subt2)/setNHS2;

      // Find the begining and the end of the current line segment.
      projPathIt = projPath.begin()+pathSeg;
      projPath1 = (*projPathIt);
      projPathIt = projPath.begin()+pathSeg+1;
      projPath2 = (*projPathIt);

      // Find the interpolated position on the projected line segment
      intPos = new Vector2d;
      intPos->x = f2*projPath2->x + (1-f2)*projPath1->x;
      intPos->y = f2*projPath2->y + (1-f2)*projPath1->y;
      projPathFull.push_back(intPos);

      tempError = (1-f)*errorRadR[in1] + f*errorRadR[in2];
      distWeightCone.push_back(tempError);
      //cout << "   Segment: " << t << ", " << steps << " pos: " << *intPos << "  error: " << tempError << "\n";
      steps += 1;
   }
   //cout << "Total size: " << projPathFull.size() << " and " << distWeightCone.size() << "\n";

   /*for(_f = projPathFull.begin(); _f != projPathFull.end(); _f++){
      cout << "point: " << *(*_f) << "\n";
   }*/
}

void advisory::buildErrorCone(){
   // Radius of the error cone using 2010 values
   //double errorRad[] = {66.672, 114.824, 157.42, 200.016, 298.172, 407.44};
   double errorRad[] = {50.004, 100.471, 144.302, 187.515, 283.263, 390.463};
   int i;

   // Avg path list
   std::vector<Vector4d*>::iterator p1;
   std::vector<Vector4d*>::iterator p2;

   Vector2d endPoint1, endPoint2, lastAvg;

   Vector2d toDraw, mapPos, * t1,  * t2;
   double tempBear, startBear, endBear, bDif, bStep;

   startBear = endBear = 0.0;

   // New std deviation using distance
   t1 = new Vector2d();
   t2 = new Vector2d();
   i = 0;
   p2 = projPath.begin()+1;

   //cout << "projPath size: " << projPath.size() << "\n";
   toDraw.set((*projPath.begin())->x, (*projPath.begin())->y);
   mapPos.set((*projPath.begin())->x, (*projPath.begin())->y);
   toDraw = translateToScreen((*projPath.begin())->x, (*projPath.begin())->y);
   stdDev1.push_back(toDraw);
   errorCone1.push_back(mapPos);
   for(p1 = projPath.begin(); p2 != projPath.end(); p1++){
      //cout << " I: " << i << endl;
      //if(i < 4){
         t1->set((*p1)->x, (*p1)->y);
         t2->set((*p2)->x, (*p2)->y);
      /*}
      else if(i == 4){
         t1->set((*p1)->x, (*p1)->y);
         t2->set((*p2)->x, (*p2)->y);
         t2->set( (((*p2)->x-(*p1)->x)/2+(*p1)->x), (((*p2)->y-(*p1)->y)/2+(*p1)->y));
      }
      else{
         t1->set((*p1)->x, (*p1)->y);
         t2->set((*p2)->x, (*p2)->y);
         t1->set( (((*p2)->x-(*p1)->x)/2+(*p1)->x), (((*p2)->y-(*p1)->y)/2+(*p1)->y));
      }*/
      tempBear = findBearing_2(t1, t2);
      tempBear = tempBear - 90.0;
      if(tempBear < 0.0){
         tempBear = tempBear + 360;
      }

      toDraw = locateDestination_2(t2->x, t2->y, errorRad[i], tempBear);

      if(p2 == projPath.end()-2){
         endPoint1.set(toDraw.x, toDraw.y);
         lastAvg.set(t2->x, t2->y);
         startBear = tempBear;
      }

      mapPos.set(toDraw.x, toDraw.y);
      toDraw = translateToScreen(toDraw.x, toDraw.y);

      stdDev1.push_back(toDraw);
      errorCone1.push_back(mapPos);
      /*if(i == 4){
         p1--;
         p2--;
      }*/
      p2++;
      i++;
      if(i == 4){
         i++;
      }
   }

   i = 0;
   p2 = projPath.begin()+1;
   toDraw.set((*projPath.begin())->x, (*projPath.begin())->y);
   mapPos.set((*projPath.begin())->x, (*projPath.begin())->y);
   toDraw = translateToScreen((*projPath.begin())->x, (*projPath.begin())->y);
   stdDev2.push_back(toDraw);
   errorCone2.push_back(mapPos);
   for(p1 = projPath.begin(); p2 != projPath.end(); p1++){
      //if(i < 4){
         t1->set((*p1)->x, (*p1)->y);
         t2->set((*p2)->x, (*p2)->y);
      /*}
      else if(i == 4){
         t1->set((*p1)->x, (*p1)->y);
         t2->set((*p2)->x, (*p2)->y);
         t2->set( (((*p2)->x-(*p1)->x)/2+(*p1)->x), (((*p2)->y-(*p1)->y)/2+(*p1)->y));
      }
      else{
         t1->set((*p1)->x, (*p1)->y);
         t2->set((*p2)->x, (*p2)->y);
         t1->set( (((*p2)->x-(*p1)->x)/2+(*p1)->x), (((*p2)->y-(*p1)->y)/2+(*p1)->y));
      }*/
      tempBear = findBearing_2(t1, t2);
      tempBear = fmod(tempBear + 90.0, 360.0);

      toDraw = locateDestination_2(t2->x, t2->y, errorRad[i], tempBear);

      if(p2 == projPath.end()-2){
         endPoint2.set(toDraw.x, toDraw.y);
         endBear = tempBear;
      }

      mapPos.set(toDraw.x, toDraw.y);
      toDraw = translateToScreen(toDraw.x, toDraw.y);

      stdDev2.push_back(toDraw);
      errorCone2.push_back(mapPos);
      /*if(i == 4){
         p1--;
         p2--;
      }*/
      p2++;
      i++;
      if(i == 4){
        i++;
      }
   }

   // Build top cone
   bDif = findDif(startBear, endBear);
   if(bDif < 0){
     cout << "Bad bDif for error cone top!!!!! " << bDif << endl << endl;
     bDif = bDif*-1;
   }
   bStep = bDif/25;

   toDraw = translateToScreen(endPoint1.x, endPoint1.y);
   top.push_back(toDraw);
   for(i = 0; i < 26; i++){
      tempBear = fmod(startBear+bStep*i, 360.0);
      toDraw = locateDestination_2(lastAvg.x, lastAvg.y, errorRad[5], startBear+bStep*i);
      toDraw = translateToScreen(toDraw.x, toDraw.y);  
      top.push_back(toDraw);
   }
   toDraw = translateToScreen(endPoint2.x, endPoint2.y);
   top.push_back(toDraw);

   // Smooth error cone curve
   std::vector<Vector4d*> nextSeg;
   std::vector<Vector2d>::iterator _ec;
   std::vector<Vector4d*>::iterator _ec2;

   Vector4d * pa, * pb;
   Vector2d q0, q1, q2;
   double degInit, degFinal;
   double s0, vX, vY;
   double ts = 9.0;
   int numSeg = 3;
   int count = 0;

   buildECSmooth();
   //buildErrorConeFull();

   delete(t1);
   delete(t2);
}

void advisory::buildErrorConeFull(){
   // Radius of the error cone using 2010 values
   double errorRad[] = {50.004, 100.471, 144.302, 187.515, 283.263, 390.463};
   int i;

   // Avg path list
   std::vector<Vector2d*>::iterator p1;
   std::vector<Vector2d*>::iterator p2;

   Vector2d endPoint1, endPoint2, lastAvg;

   Vector2d toDraw, * t1,  * t2;
   double tempBear, endBear;

   // New std deviation using distance
   t1 = new Vector2d();
   t2 = new Vector2d();
   i = 0;
   p2 = projPathFull.begin()+1;

   //cout << "projPath size: " << projPath.size() << "\n";
   toDraw.set((*projPathFull.begin())->x, (*projPathFull.begin())->y);
   errorCone1.push_back(toDraw);
   toDraw = translateToScreen((*projPath.begin())->x, (*projPath.begin())->y);
   stdDev1.push_back(toDraw);
   for(p1 = projPathFull.begin(); p2 != projPathFull.end(); p1++){
      t1->set((*p1)->x, (*p1)->y);
      t2->set((*p2)->x, (*p2)->y);

      tempBear = findBearing_2(t1, t2);
      tempBear = tempBear - 90.0;
      if(tempBear < 0.0){
         tempBear = tempBear + 360;
      }

      toDraw = locateDestination_2(t2->x, t2->y, distWeightCone[i], tempBear);
      errorCone1.push_back(toDraw);

      p2++;
      i++;
   }

   i = 0;
   p2 = projPathFull.begin()+1;
   toDraw.set((*projPathFull.begin())->x, (*projPathFull.begin())->y);
   errorCone2.push_back(toDraw);
   for(p1 = projPathFull.begin(); p2 != projPathFull.end(); p1++){
      t1->set((*p1)->x, (*p1)->y);
      t2->set((*p2)->x, (*p2)->y);
      tempBear = findBearing_2(t1, t2);
      tempBear = fmod(tempBear + 90.0, 360.0);

      toDraw = locateDestination_2(t2->x, t2->y, errorRad[i], tempBear);
      errorCone2.push_back(toDraw);

      if(p2 == projPathFull.end()-1){
         endPoint2.set(toDraw.x, toDraw.y);
         endBear = tempBear;
      }

      toDraw = translateToScreen(toDraw.x, toDraw.y);

      p2++;
      i++;
   }

   delete(t1);
   delete(t2);
}

void advisory::findNewAvg(){
   int k;
   unsigned int i, j;
   int count;
   Vector2d * toPush, * toPushLatLon, * pushAvgSegVel;
   std::vector<path*>::iterator _p;
   numIn68 = (int)floor(pathList.size()*0.8);

   if(numIn68 < 1){
      numIn68 = 1;
   }

   adv = projPath[4];
   adv2 = projPath[5];
   sortInd = 22;
   std::sort(pathList.begin(), pathList.end(), sp);

   for(i = 0; i < newAvg.size(); i++){
      delete(newAvg[i]);
   }
   for(i = 0; i < avgPathLatLon.size(); i++){
      delete(avgPathLatLon[i]);
   }
   for(i = 0; i < avgSegVel.size(); i++){
      delete(avgSegVel[i]);
   }

   newAvg.clear();
   avgPathLatLon.clear();
   avgSegVel.clear();
   avgPath3.clear();
   avgSegBearing.clear();
   avgSegDist.clear();

   closestToPre.clear();
   _p = pathList.begin();
   for(k = 0; k < numIn68; k++){
      closestToPre.push_back((*_p));
      _p++;
   }

   _p = closestToPre.begin();
   while( newAvg.size() < (*_p)->posList.size()){
      toPush = new Vector2d(0.0, 0.0);
      newAvg.push_back(toPush);
      toPushLatLon = new Vector2d(0.0, 0.0);
      avgPathLatLon.push_back(toPushLatLon);
   }

   for (i = 0; i < closestToPre.size(); i++) {
      for (j = 0; j != closestToPre[i]->posList.size(); j++) {
         newAvg[j]->x = newAvg[j]->x + closestToPre[i]->posList[j]->x;
         newAvg[j]->y = newAvg[j]->y + closestToPre[i]->posList[j]->y;
         avgPathLatLon[j]->x = avgPathLatLon[j]->x + closestToPre[i]->latLonList[j]->x;
         avgPathLatLon[j]->y = avgPathLatLon[j]->y + closestToPre[i]->latLonList[j]->y;
      }
   }

   // Find average now
   count = closestToPre.size();
   for( i = 0; i < newAvg.size(); i++){
      newAvg[i]->x =  newAvg[i]->x/(double)count;
      newAvg[i]->y =  newAvg[i]->y/(double)count;
      avgPathLatLon[i]->x = avgPathLatLon[i]->x/count;
      avgPathLatLon[i]->y = avgPathLatLon[i]->y/count;
   }


   // Find average velocity for every segment
   double tBearing, tDist, aDist;
   Vector2d curLoc, tNewLoc;

   for(i = 0; i < closestToPre[0]->posList.size()-1; i++){
      tBearing = 0.0;
      aDist = 0.0;
      pushAvgSegVel = new Vector2d(0.0, 0.0);
      for(j = 0; j < closestToPre.size(); j++){
         tBearing = findBearing_2(closestToPre[j]->latLonList[i], closestToPre[j]->latLonList[i+1]);
         tDist = haversine( closestToPre[j]->latLonList[i]->x, closestToPre[j]->latLonList[i+1]->x,
                            closestToPre[j]->latLonList[i]->y, closestToPre[j]->latLonList[i+1]->y);
         pushAvgSegVel->x = pushAvgSegVel->x + sin((tBearing*M_PI)/180.0)*tDist;
         pushAvgSegVel->y = pushAvgSegVel->y + cos((tBearing*M_PI)/180.0)*tDist;
         aDist += tDist;
      }
      pushAvgSegVel->set(*pushAvgSegVel/closestToPre.size());
      aDist = aDist/closestToPre.size();
      avgSegDist.push_back(aDist);
      avgSegVel.push_back(pushAvgSegVel);
      /*cout << "Avg Vel for Seg " << i << ": " << *pushAvgSegVel << "\n";
      cout << "   Avg Bearing: " << aBearing << "\n";
      cout << "   Avg Dist: " << aDist << "\n";*/
   }

   //avgPath.clear();
   avgPathLatLon.clear();

   //cout << "avgSegVel size: " << avgSegDist.size() << "\n";
   toPushLatLon = new Vector2d(avgPathLatLon[0]->x, avgPathLatLon[0]->y);
   avgPathLatLon.clear();
   //cout << "   New avg pos for seg 0: " << *toPushLatLon << "\n";
   avgPath3.push_back(toPushLatLon);
   avgPathLatLon.push_back(toPushLatLon);
   curLoc.set(avgPathLatLon[0]->x, avgPathLatLon[0]->y);
   for(i = 0; i < avgSegDist.size(); i++){
      toPushLatLon = new Vector2d;
      tBearing = findDeg(avgSegVel[i]->normalize().x, avgSegVel[i]->normalize().y);
      //cout << "bearing for seg " << i << ": " << tBearing << " from vel: " <<  avgSegVel[i]->normalize() << "   with dist: " << avgSegDist[i] << "\n";
      tNewLoc = locateDestination_2(curLoc.x, curLoc.y, avgSegDist[i], tBearing);
      toPushLatLon->set(tNewLoc.x, tNewLoc.y);
      //cout << "   New avg pos for seg " << i+1 << ": " << *toPushLatLon << "\n";
      avgPath3.push_back(toPushLatLon);
      avgPathLatLon.push_back(toPushLatLon);
      avgSegBearing.push_back(tBearing);
      curLoc.set(tNewLoc.x, tNewLoc.y);
   }

   //cout << "New avg path size: " << avgPath3.size() << "\n";

   // Now do the std deviation
   newStdDeviationLine();
}

void advisory::newStdDeviationLine(){
   unsigned int i, j;
   int k;
   std::vector<Vector2d*>::iterator pos;
   std::vector<Vector2d*>::iterator devListIt;
   std::vector<path*>::iterator _p;
   int pct = (int)floor(pathList.size()*0.8);
   Vector2d * curDev;
   double distDev;

   stdDevDist.clear();

   if(pct < 1){
      pct = 1;
   }

   closestToPreStdDev.clear();
   _p = pathList.begin();
   for(k = 0; k < pct; k++){
      closestToPreStdDev.push_back((*_p));
      _p++;
   }

   // Compute std deviation for one segment at a time
   curDev = new Vector2d(0.0, 0.0);
   for (j = 1; j != closestToPreStdDev[0]->posList.size(); j++) {
      curDev->set(0.0, 0.0);
      distDev = 0.0;
      for (i = 0; i < closestToPreStdDev.size(); i++) {
         curDev->x = curDev->x + Sqr(closestToPreStdDev[i]->posList[j]->x - newAvg[j]->x);
         curDev->y = curDev->y + Sqr(closestToPreStdDev[i]->posList[j]->y - newAvg[j]->y);
         distDev = distDev + Sqr(haversine(closestToPreStdDev[i]->latLonList[j]->x, avgPathLatLon[j]->x, closestToPreStdDev[i]->latLonList[j]->y, avgPathLatLon[j]->y));
      }
      curDev->x = sqrt((1.0/closestToPreStdDev.size())*curDev->x);
      curDev->y = sqrt((1.0/closestToPreStdDev.size())*curDev->y);
      distDev = sqrt((1.0/closestToPreStdDev.size())*distDev);
      //cout << "distDev: " << distDev << "\n";
      stdDevDist.push_back(1.5*distDev);
   }
   delete(curDev);
}


// -------------------------- Statistical Functions --------------------------//

// Find the average line of the generated paths for each advisory
void advisory::findAvgLine(){
   std::vector<path*>::iterator pathIt;
   int count = pathList.size();
   int k;
   unsigned int i, j;
   std::vector<path*>::iterator _p;
   std::vector<Vector2d*>::iterator pos1;
   Vector2d * toPush;
   Vector2d * toPushLatLon;
   Vector2d * pushAvgSegVel;
   numIn68 = (int)floor(pathList.size()*0.68);

   if(numIn68 < 1){
      numIn68 = 1;
   }

   adv = projPath[4];
   adv2 = projPath[5];
   std::sort(pathList.begin(), pathList.end(), sp);

   for(i = 0; i < avgPath.size(); i++){
      delete(avgPath[i]);
   }
   for(i = 0; i < avgPathLatLon.size(); i++){
      delete(avgPathLatLon[i]);
   }
   for(i = 0; i < avgSegVel.size(); i++){
      delete(avgSegVel[i]);
   }
   /*for(i = 0; i < avgPath3.size(); i++){
      delete(avgPath3[i]);
   }*/

   avgPath.clear();
   avgPathLatLon.clear();
   avgSegVel.clear();
   avgPath3.clear();
   avgSegBearing.clear();
   avgSegDist.clear();
   pathIt = pathList.begin();

   closestToPre.clear();
   _p = pathList.begin();
   for(k = 0; k < numIn68; k++){
      closestToPre.push_back((*_p));
      _p++;
   }

   // Make sure that the avgPath has enough spots
   while( avgPath.size() < (*pathIt)->posList.size()){
      toPush = new Vector2d(0.0, 0.0);
      avgPath.push_back(toPush);
      toPushLatLon = new Vector2d(0.0, 0.0);
      avgPathLatLon.push_back(toPushLatLon);
   }

   for (i = 0; i < pathList.size(); i++) {
      for (j = 0; j != pathList[i]->posList.size(); j++) {
         avgPath[j]->x = avgPath[j]->x + pathList[i]->posList[j]->x;
         avgPath[j]->y = avgPath[j]->y + pathList[i]->posList[j]->y;
         avgPathLatLon[j]->x = avgPathLatLon[j]->x + pathList[i]->latLonList[j]->x;
         avgPathLatLon[j]->y = avgPathLatLon[j]->y + pathList[i]->latLonList[j]->y;
      }
   }

   // Find average now
   for( i = 0; i < avgPath.size(); i++){
      avgPath[i]->x =  avgPath[i]->x/count;
      avgPath[i]->y =  avgPath[i]->y/count;
      //cout << "avg Path: " << avgPath[i]->x << ", " << avgPath[i]->y << " with count: " << count << "\n";
      avgPathLatLon[i]->x = avgPathLatLon[i]->x/count;
      avgPathLatLon[i]->y = avgPathLatLon[i]->y/count;
   }

   // Find average velocity for every segment
   double tBearing, tDist, aDist;
   Vector2d curLoc, tNewLoc;

   for(i = 0; i < pathList[0]->posList.size()-1; i++){
      tBearing = 0.0;
      aDist = 0.0;
      pushAvgSegVel = new Vector2d(0.0, 0.0);
      for(j = 0; j < pathList.size(); j++){
         tBearing = findBearing_2(pathList[j]->latLonList[i], pathList[j]->latLonList[i+1]);
         tDist = haversine( pathList[j]->latLonList[i]->x, pathList[j]->latLonList[i+1]->x,
                            pathList[j]->latLonList[i]->y, pathList[j]->latLonList[i+1]->y);
         pushAvgSegVel->x = pushAvgSegVel->x + sin((tBearing*M_PI)/180.0)*tDist;
         pushAvgSegVel->y = pushAvgSegVel->y + cos((tBearing*M_PI)/180.0)*tDist;
         aDist += tDist;
      }
      pushAvgSegVel->set(*pushAvgSegVel/pathList.size());
      aDist = aDist/pathList.size();
      avgSegDist.push_back(aDist);
      avgSegVel.push_back(pushAvgSegVel);
      /*cout << "Avg Vel for Seg " << i << ": " << *pushAvgSegVel << "\n";
      cout << "   Avg Bearing: " << aBearing << "\n";
      cout << "   Avg Dist: " << aDist << "\n";*/
   }

   //avgPath.clear();
   avgPathLatLon.clear();

   //cout << "avgSegVel size: " << avgSegDist.size() << "\n";
   toPushLatLon = new Vector2d(avgPathLatLon[0]->x, avgPathLatLon[0]->y);
   avgPathLatLon.clear();
   //cout << "   New avg pos for seg 0: " << *toPushLatLon << "\n";
   avgPath3.push_back(toPushLatLon);
   avgPathLatLon.push_back(toPushLatLon);
   curLoc.set(avgPathLatLon[0]->x, avgPathLatLon[0]->y);
   for(i = 0; i < avgSegDist.size(); i++){
      toPushLatLon = new Vector2d;
      tBearing = findDeg(avgSegVel[i]->normalize().x, avgSegVel[i]->normalize().y);
      //cout << "bearing for seg " << i << ": " << tBearing << " from vel: " <<  avgSegVel[i]->normalize() << "   with dist: " << avgSegDist[i] << "\n";
      tNewLoc = locateDestination_2(curLoc.x, curLoc.y, avgSegDist[i], tBearing);
      toPushLatLon->set(tNewLoc.x, tNewLoc.y);
      //cout << "   New avg pos for seg " << i+1 << ": " << *toPushLatLon << "\n";
      avgPath3.push_back(toPushLatLon);
      avgPathLatLon.push_back(toPushLatLon);
      avgSegBearing.push_back(tBearing);
      curLoc.set(tNewLoc.x, tNewLoc.y);
   }

   //cout << "New avg path size: " << avgPath3.size() << "\n";

   // Now do the std deviation
   stdDeviationLine();

   //********************************//
   // Rebuild avg to throw out outliers
   avgPath2.clear();
   avgPathLatLon2.clear();
   pathIt = pathList.begin();
   int c = 0;
   int l;

   // Make sure that the avgPath has enough spots
   while( avgPath2.size() < (*pathIt)->posList.size()){
      toPush = new Vector2d(0.0, 0.0);
      avgPath2.push_back(toPush);
      toPushLatLon = new Vector2d(0.0, 0.0);
      avgPathLatLon2.push_back(toPushLatLon);
   }

   for (i = 0; i < pathList.size(); i++) {
      l = stdDevDist.size()-1;
      if(haversine(pathList[i]->latLonList[l]->x, projPath[5]->x, pathList[i]->latLonList[l]->y, projPath[5]->y) < stdDevDist[l]){
         c++;
         for (j = 0; j != pathList[i]->posList.size(); j++) {
            avgPath2[j]->x = avgPath2[j]->x + pathList[i]->posList[j]->x;
            avgPath2[j]->y = avgPath2[j]->y + pathList[i]->posList[j]->y;
            //cout << "j: " << j << " with " << pathList[i]->posList[j]->x << " and " << pathList[i]->posList[j]->y << "\n";
            avgPathLatLon2[j]->x = avgPathLatLon2[j]->x + pathList[i]->latLonList[j]->x;
            avgPathLatLon2[j]->y = avgPathLatLon2[j]->y + pathList[i]->latLonList[j]->y;
         }
      }
   }

   // Find average now
   for( i = 0; i < avgPath.size(); i++){
      avgPath2[i]->x =  avgPath2[i]->x/c;
      avgPath2[i]->y =  avgPath2[i]->y/c;
      //cout << "avg Path: " << avgPath2[i]->x << ", " << avgPath2[i]->y << " with count: " << c << "\n";
      avgPathLatLon2[i]->x = avgPathLatLon2[i]->x/c;
      avgPathLatLon2[i]->y = avgPathLatLon2[i]->y/c;
   }

   // Find the error of speed and distance between avg line and projected path
   if(avgPath.size() > 0 && pathList.size() > 25){
      //cout << "BAM!\n";
      speedDistBearingError();
   }
}

void advisory::stdDeviationLine(){
   unsigned int i, j;
   std::vector<Vector2d*>::iterator pos;
   std::vector<Vector2d*>::iterator devListIt;
   Vector2d * curDev;
   double distDev;

   stdDevDist.clear();

   // Compute std deviation for one segment at a time
   curDev = new Vector2d(0.0, 0.0);
   for (j = 1; j != pathList[0]->posList.size(); j++) {
      curDev->set(0.0, 0.0);
      distDev = 0.0;
      for (i = 0; i < pathList.size(); i++) {
         curDev->x = curDev->x + Sqr(pathList[i]->posList[j]->x - avgPath[j]->x);
         curDev->y = curDev->y + Sqr(pathList[i]->posList[j]->y - avgPath[j]->y);
         distDev = distDev + Sqr(haversine(pathList[i]->latLonList[j]->x, avgPathLatLon[j]->x, pathList[i]->latLonList[j]->y, avgPathLatLon[j]->y));
      }
      curDev->x = sqrt((1.0/pathList.size())*curDev->x);
      curDev->y = sqrt((1.0/pathList.size())*curDev->y);
      distDev = sqrt((1.0/pathList.size())*distDev);
      //cout << "distDev: " << distDev << "\n";
      stdDevDist.push_back(sim->getSDA()*distDev);
   }
   delete(curDev);
}

void advisory::speedDistBearingError(){
   // Equation variables
   int n = 0;              // How many time steps we've taken
   double t;                  // Current elapsed time
   int i;                     // Which time frame we are looking at
   double f;
   double dist, distRat;
   double intPosLat, intPosLon, intPosLatStart, intPosLonStart;
   double subt, post;
   double setNHS;
   double speed, speed2, speedF;
   double bearingAvg, bearingProj, bearingDif, tempBearing;

   Vector4d * projPath1, * projPath2;
   Vector2d temp, temp2, * tPos1, * tPos2, tempError;
   Vector2d velP, velA, velE;
   std::vector<Vector4d*>::iterator projPathIt;
   std::vector<path*>::iterator _p;

   distError.clear();
   distError2.clear();
   speedError.clear();
   bearingError.clear();
   avgBearing.clear();
   projBearing.clear();
   speedAvg.clear();
   velError.clear();
   velAvg.clear();
   distRatio.clear();
   predVel.clear();

   intPosLonStart = (*projPath.begin())->x;
   intPosLatStart = (*projPath.begin())->y;

   _p = pathList.begin();
   for(n = 0; n < (*_p)->size(); n++){
      t = n*sim->getHours();
      // Find i, subt and setNHS based on the hours completed so far
      if(t < 9){
         i = 0;
         subt = 0;
         post = 9;
         setNHS = 9;
      }
      else if(t < 21){
         i = 1;
         subt = 9;
         setNHS = 12;
         post = 21;
      }
      else if(t < 33){
         i = 2;
         subt = 21;
         setNHS = 12;
         post = 33;
      }
      else if(t < 45){
         i = 3;
         subt = 33;
         setNHS = 12;
         post = 45;
      }
      else{
         i = 4;
         subt = 45;
         setNHS = 24;
         post = 69;
      }

      // Catch for the last time step
      if(post - t == 0){
         post = 93;
      }

      // Find the weight value
      f = (t - subt)/setNHS;

      // Find the begining and the end of the current line segment.
      projPathIt = projPath.begin()+i;
      projPath1 = (*projPathIt);
      projPathIt = projPath.begin()+i+1;
      projPath2 = (*projPathIt);

      // Find the interpolated position on the projected line segment
      intPosLon = f*projPath2->x + (1-f)*projPath1->x;
      intPosLat = f*projPath2->y + (1-f)*projPath1->y;

      //cout << "Using proj path points: " << *projPath1 << " and " << *projPath2 << " for hours: " << n*hours << "\n";
      //cout << " current f: " << f << "\n";
      //cout << " interpolated start: " << intPosLonStart << " / " << intPosLatStart << "\n";
      //cout << " for interpolated lat/lon: " << intPosLon << " / " << intPosLat << "\n";

      // Find interpolated std deviation

      // Find dist from avg line at this point to the interpolated position on the projected path
      dist = haversine(intPosLon, avgPathLatLon[n]->x, intPosLat, avgPathLatLon[n]->y);
      tPos1 = new Vector2d;
      tPos2 = new Vector2d;
      tPos1->set(avgPathLatLon[n]->x, avgPathLatLon[n]->y);
      tPos2->set(intPosLon, intPosLat);
      //tempBearing = findBearing_2(tPos1, tPos2);
      tempBearing = findBearing_2(tPos2, tPos1);
      tempError.x = sin(tempBearing*M_PI/180.0)*dist;
      tempError.y = cos(tempBearing*M_PI/180.0)*dist;
      //cout << "t: " << t << "    n: " << n << "    dist: " << dist;
      distError.push_back(tempError);
      distError2.push_back(dist);

      delete(tPos1);
      delete(tPos2);

      // Find the difference in bearing between this segment and the interpolated position on the projected path
      // Computed per segment, not per point.
      _p = pathList.begin();
      if(n < (*_p)->size()-1){
         // Find the difference in speed between this point to the interpolated position on the projected path
         //cout << " projPath2: " << *projPath2 << "\n";
         speed = (haversine(intPosLon, projPath2->x, intPosLat, projPath2->y)/(post-t))*sim->getHours();
         speed2 = haversine(avgPathLatLon[n]->x, avgPathLatLon[n+1]->x,
               avgPathLatLon[n]->y, avgPathLatLon[n+1]->y);
         //cout << "Segment: " << t/3 << " with post: " << post << " t: " << t << "  hours: " << hours << " dividing by: " << (post-t) << "\n";
         //cout << "     Avg Path: " << *avgPathLatLon[n] << " to " << *avgPathLatLon[n+1] << "\n";
         //cout << "     Proj Speed: " << speed << "    Avg Speed: " << speed2 << "\n";
         speedAvg.push_back(speed2);
         speedF = speed2 - speed;
         //cout << "     speedDif: " << speed << "\n";
         //speedError.push_back(speedF);

         distRat = speed/speed2;
         distRatio.push_back(distRat);
         //cout << "Pushing dist ratio: " << distRat;

         tPos1 = new Vector2d;
         tPos2 = new Vector2d;
         tPos1->set(intPosLon, intPosLat);
         tPos2->set(projPath2->x, projPath2->y);
         bearingProj = findBearing_2(tPos1, tPos2);
         //cout << "   Projected bearing: " << bearingProj << "\n";
         projBearing.push_back(bearingProj);

         velP.x = sin(bearingProj*M_PI/180.0)*speed;
         velP.y = cos(bearingProj*M_PI/180.0)*speed;
         //cout << "   Projected vel: " << velP << "\n";


         tPos1->set(avgPathLatLon[n]->x, avgPathLatLon[n]->y);
         tPos2->set(avgPathLatLon[n+1]->x, avgPathLatLon[n+1]->y);
         bearingAvg = findBearing_2(tPos1, tPos2);
         avgBearing.push_back(bearingAvg);
         velA.x = sin(bearingAvg*M_PI/180.0)*speed2;
         velA.y = cos(bearingAvg*M_PI/180.0)*speed2;
         velAvg.push_back(velA);

         velE = velP - velA;
         //velE = velP.normalize() - velA.normalize();
         velError.push_back(velE);
         predVel.push_back(velP);
         speedError.push_back(velA.norm()-velP.norm());
         //cout << "Segment: " << n << "\n";
         //cout << "   Avg Bearing: " << bearingAvg << "\n";
         //cout << "   Avg vel: " << velA << "\n";
         //cout << "VelP: " << velP << "    VelE: " << velE << "     VelA: " << velA << "\n";
         bearingDif = bearingProj-bearingAvg;
         double mod = 1;
         if(abs(bearingDif) > 180.0){
            if(bearingDif < 0.0){
               mod = -1;
            }
            bearingAvg = 360.0-bearingAvg;
            bearingDif = bearingProj-bearingAvg*mod;
         }
         //cout << "    bearingProj: " << bearingProj << "    bearingAvg: " << bearingAvg << "    bearingDif: " << bearingDif << "\n";
         bearingError.push_back(bearingDif);

         intPosLonStart = intPosLon;
         intPosLatStart = intPosLat;
      }
      //delete(tPos1);
      //delete(tPos2);
   }
}

// ----------------------------- Drawing Functions ---------------------------//

void advisory::drawGenPaths(){
   std::vector<Vector2d*>::iterator pos1;
   std::vector<Vector2d*>::iterator pos2;
   std::vector<path*>::iterator pathIt;
   Vector2d draw1, draw2;
   float lineWidth;
   float tempTrans;

   // Values for testing whether the median sort is correct
   int side = 0;
   int med = (int)(pathList.size()/2.0);
   adv = projPath[4];
   adv2 = projPath[5];
   sortPath(22);

   onRight = 0;
   onLeft = 0;

   // Display generated paths
   for (pathIt = pathList.begin(); pathIt != pathList.end(); pathIt++){
      int testSide = 0;
      pos2 = (*pathIt)->posList.begin() + 1;
      lineWidth = 1.0;
      //glColor4f((*pathIt)->getInt().x, (*pathIt)->getInt().y,
      //      (*pathIt)->getInt().z, (*pathIt)->getInt().w);
      tempTrans = (*pathIt)->getInt().w*sim->getPathOpacity();
      //cout << "Path size: " << (*pathIt)->posList2.size() << endl;
      for (pos1 = (*pathIt)->posList.begin(); pos2 != (*pathIt)->posList.end(); pos1++) {
         //glColor4f(tempTrans*(*pathIt)->getInt().x, tempTrans*(*pathIt)->getInt().y,
         //      tempTrans*(*pathIt)->getInt().z, tempTrans);
         glColor4f(tempTrans*1.0, tempTrans*0.0,
               tempTrans*0.0, tempTrans);
         //if(side <= med){
         if((*pathIt)->getSide() > 0){
            //glColor4f(0.2, 0.2, 1.0, tempTrans);
            if(pos2+1 == (*pathIt)->posList.end()){
               testSide--;
            }
            //glColor4f(0.0, 0.0, 1.0, tempTrans);
            glColor4f(tempTrans*0.0, tempTrans*0.0,
               tempTrans*1.0, tempTrans);
         }
         else if((*pathIt)->getSide() < 0){
            //glColor4f(0.2, 0.2, 1.0, tempTrans);
            if(pos2+1 == (*pathIt)->posList.end()){
               testSide++;
            }
            glColor4f(tempTrans*0.0, tempTrans*0.0,
               tempTrans*1.0, tempTrans);
            //glColor4f(0.0, 0.0, 1.0, tempTrans);
            //glColor4f(1.0, 0.0, 0.0, tempTrans);
         }
         else{
            glColor4f(0.0, 0.0, 0.0, 0.0);
         }
         glLineWidth(lineWidth * 1.0);
         draw1.x = (*pos1)->x;
         draw1.y = (*pos1)->y;
         draw2.x = (*pos2)->x;
         draw2.y = (*pos2)->y;
         glBegin(GL_LINES);
            glVertex3f(draw1.x, draw1.y, 0.0);
            glVertex3f(draw2.x, draw2.y, 0.0);
         glEnd();
         pos2++;
         //lineWidth = lineWidth + 0.5;
         //tempTrans = tempTrans - 0.025;
      }

      testSide < 0 ? onLeft++ : onRight++;
      side++;
      (*pathIt)->setInt((*pathIt)->getInt().w - 0.001);
   }
   for (pathIt = pathList.begin(); pathIt != pathList.end();){
      if ((*pathIt)->getInt().w <= 0.0) {
         delete((*pathIt));
         pathIt = pathList.erase(pathIt);
      }
      else{
         pathIt++;
      }
   }

   //cout << "OnLeft:  " << onLeft << "     OnRight: " << onRight << endl;
}

void advisory::drawGenPathsClosest(){
   std::vector<Vector2d*>::iterator pos1;
   std::vector<Vector2d*>::iterator pos2;
   std::vector<path*>::iterator pathIt;
   Vector2d draw1, draw2;
   float lineWidth;
   float tempTrans;

   // Display generated paths
   for (pathIt = closestToPre.begin(); pathIt != closestToPre.end(); pathIt++){
      int test = 0;
      pos2 = (*pathIt)->posList.begin() + 1;
      lineWidth = 1.20;
      glColor4f((*pathIt)->getInt().x, (*pathIt)->getInt().y,
            (*pathIt)->getInt().z, (*pathIt)->getInt().w);
      tempTrans = (*pathIt)->getInt().w*0.4;
      //cout << "Path size: " << (*pathIt)->posList2.size() << endl;
      for (pos1 = (*pathIt)->posList.begin(); pos2 != (*pathIt)->posList.end(); pos1++) {
         //glColor4f((*pathIt)->getInt().x, (*pathIt)->getInt().y,
         //      (*pathIt)->getInt().z, tempTrans);
         if(test%2 == 0){
            glColor4f(1.0, 0.2, 0.2, tempTrans);
            //glColor4f(0.0, 0.0, 1.0, tempTrans);
         }
         else{
            glColor4f(1.0, 0.2, 0.2, tempTrans);
            //glColor4f(1.0, 0.0, 0.0, tempTrans);
         }
         glLineWidth(lineWidth * 1.0);
         draw1.x = (*pos1)->x;
         draw1.y = (*pos1)->y;
         draw2.x = (*pos2)->x;
         draw2.y = (*pos2)->y;
         glBegin(GL_LINES);
            glVertex3f(draw1.x, draw1.y, 0.0);
            glVertex3f(draw2.x, draw2.y, 0.0);
         glEnd();
         pos2++;
         test++;
         //lineWidth = lineWidth + 0.5;
         //tempTrans = tempTrans - 0.025;
      }
   }
}

void advisory::drawHeatMap(){
   std::vector<Vector2d*>::iterator pos1;
   std::vector<Vector2d*>::iterator pos2;
   std::vector<path*>::iterator pathIt;
   Vector2d draw1, draw2;
   float lineWidth;

   pathIt = pathList.end();

   for (pathIt = pathList.begin(); pathIt != pathList.end(); pathIt++) {
      int test = 0;
      //cout << "Path size: " << (*pathIt)->posList.size() << "\n";
      pos2 = (*pathIt)->posList.begin() + 1;
      lineWidth = 1.0;
      glColor4f(1.0, 1.0, 1.0, 0.01);
      for (pos1 = (*pathIt)->posList.begin(); pos2 != (*pathIt)->posList.end();
            pos1++) {
         glLineWidth(lineWidth * 1.0);
         draw1.x = (*pos1)->x;
         draw1.y = (*pos1)->y;
         draw2.x = (*pos2)->x;
         draw2.y = (*pos2)->y;
         //cout << "Line : " << draw1 << " to " << draw2 << "\n";
         glBegin(GL_LINES);
         glVertex3f(draw1.x, draw1.y, 0.0);
         glVertex3f(draw2.x, draw2.y, 0.0);
         glEnd();
         pos2++;
         test++;
      }
   }
}

// Draw the average path of the generated paths for each advisory in the simulation
void advisory::drawAvgPath(){
   std::vector<Vector2d*>::iterator p1;
   std::vector<Vector2d*>::iterator p2;
   Vector2d toDraw1, toDraw2;

   glColor4f(0.0, 1.0, 0.0, 1.0);
   glLineWidth(2.5);

   p2 = avgPathLatLon.begin()+1;
   for(p1 =avgPathLatLon.begin(); p2 != avgPathLatLon.end(); p1++){
      glBegin(GL_LINES);
         toDraw1 = translateToScreen((*p1)->x, (*p1)->y);
         toDraw2 = translateToScreen((*p2)->x, (*p2)->y);
         glVertex3f(toDraw1.x, toDraw1.y, 0.0);
         glVertex3f(toDraw2.x, toDraw2.y, 0.0);
      glEnd();
      p2++;
   }

   glColor4f(1.0, 0.0, 1.0, 1.0);
   double a, b;
   a = 1.0;
   b = 0.0;
   p2 = avgPath3.begin()+1;
    for(p1 = avgPath3.begin(); p2 != avgPath3.end(); p1++){
       glColor4f(a, 0.0, b, 1.0);
       glBegin(GL_LINES);
          toDraw1 = translateToScreen((*p1)->x, (*p1)->y);
          toDraw2 = translateToScreen((*p2)->x, (*p2)->y);
          glVertex3f(toDraw1.x, toDraw1.y, 0.0);
          glVertex3f(toDraw2.x, toDraw2.y, 0.0);
       glEnd();
       p2++;
       if(a == 1.0){
          a = 0.0;
          b = 1.0;
       }
       else{
          a = 1.0;
          b = 0.0;
       }
    }
}

// Draw the projected path for each advisory in the simulation
void advisory::drawForecastPath() {
   Vector2d draw1, draw2;
   std::vector<Vector4d*>::iterator pos1;
   std::vector<Vector4d*>::iterator pos2;

   glColor4f(1.0, 0.0, 0.0, 1.0);
   glLineWidth(1.0);

   pos2 = projPath.begin() + 1;
   for (pos1 = projPath.begin(); pos2 != projPath.end()-1; pos1++) {
      draw1 = translateToScreen((*pos1)->x, (*pos1)->y);
      draw2 = translateToScreen((*pos2)->x, (*pos2)->y);
      glBegin(GL_LINES);
      glVertex3f(draw1.x, draw1.y, 0.0);
      glVertex3f(draw2.x, draw2.y, 0.0);
      glEnd();
      pos2++;
   }
   drawErrorCone();
}

// Draw the error cones of a projected path
void advisory::drawErrorCone(){
   Vector2d draw1, draw2;
   std::vector<Vector2d>::iterator pos1;
   std::vector<Vector2d>::iterator pos2;
   std::vector<Vector4d*>::iterator pos1_4d;
   std::vector<Vector4d*>::iterator pos2_4d;

   glColor4f(1.0, 0.0, 0.0, 0.5);
   glLineWidth(1.5);

   // Draw First Side
   pos2 = stdDev1.begin() + 1;
   for (pos1 = stdDev1.begin(); pos2 != stdDev1.end(); pos1++) {
      draw1 = (*pos1);
      draw2 = (*pos2);
      glBegin(GL_LINES);
      glVertex3f(draw1.x, draw1.y, 0.0);
      glVertex3f(draw2.x, draw2.y, 0.0);
      glEnd();
      pos2++;
   }

   // Draw Second Side
   pos2 = stdDev2.begin() + 1;
   for (pos1 = stdDev2.begin(); pos2 != stdDev2.end(); pos1++) {
      draw1 = (*pos1);
      draw2 = (*pos2);
      glBegin(GL_LINES);
      glVertex3f(draw1.x, draw1.y, 0.0);
      glVertex3f(draw2.x, draw2.y, 0.0);
      glEnd();
      pos2++;
   }

   // Draw Connecting Lines
   pos2 = stdDev2.begin();
   for (pos1 = stdDev1.begin(); pos1 != stdDev1.end(); pos1++) {
      if(pos1 != stdDev1.end()-2){
         draw1 = (*pos1);
         draw2 = (*pos2);
         glBegin(GL_LINES);
         glVertex3f(draw1.x, draw1.y, 0.0);
         glVertex3f(draw2.x, draw2.y, 0.0);
         glEnd();
      }
      pos2++;
   }

   // Draw the rounded top
   pos2 = top.begin()+1;
   for(pos1 = top.begin(); pos1 != top.end()-1; pos1++){
      draw1 = (*pos1);
      draw2 = (*pos2);
      glBegin(GL_LINES);
      glVertex3f(draw1.x, draw1.y, 0.0);
      glVertex3f(draw2.x, draw2.y, 0.0);
      glEnd();
      pos2++;
   }

   // Draw the smoothed side
   glColor4f(0.0, 0.0, 1.0, 0.5);
   pos2_4d = ecSmooth1.begin()+1;
   for(pos1_4d = ecSmooth1.begin(); pos2_4d != ecSmooth1.end(); pos1_4d++){
      draw1 = translateToScreen((*pos1_4d)->x, (*pos1_4d)->y);
      draw2 = translateToScreen((*pos2_4d)->x, (*pos2_4d)->y);
      glBegin(GL_LINES);
      glVertex3f(draw1.x, draw1.y, 0.0);
      glVertex3f(draw2.x, draw2.y, 0.0);
      glEnd();
      pos2_4d++;
   }
}

void advisory::drawStdDevPath(){
   // Avg path list
   std::vector<Vector2d*>::iterator p1;
   std::vector<Vector2d*>::iterator p2;
   // Std deviation list
   std::vector<double>::iterator d1;

   Vector2d toDraw1, toDrawPrev1, toDraw2, toDrawPrev2, t1, t2;
   double tempBear;
   int i = 0;

   glColor4f(0.0, 1.0, 0.0, 1.0);
   glLineWidth(2.5);

   // New std deviation using distance
   d1 = stdDevDist.begin();
   p2 = avgPathLatLon.begin()+1;

   toDrawPrev1 = translateToScreen( (*avgPathLatLon.begin())->x, (*avgPathLatLon.begin())->y);
   toDrawPrev2 = toDrawPrev1;
   for(p1 = avgPathLatLon.begin(); p2 != avgPathLatLon.end(); p1++){
      if(i == 0 || i == 2 || i == 6 || i == 10 || i == 14 || i == 22){
         tempBear = findBearing_2((*p1), (*p2));
         tempBear = tempBear - 90.0;
         if(tempBear < 0.0){
            tempBear = tempBear + 360;
         }

         toDraw1 = locateDestination_2((*p2)->x, (*p2)->y, (*d1), tempBear);
         toDraw1 = translateToScreen(toDraw1.x, toDraw1.y);

         glBegin(GL_LINES);
            glVertex3f(toDrawPrev1.x, toDrawPrev1.y, 0.0);
            glVertex3f(toDraw1.x, toDraw1.y, 0.0);
         glEnd();

         tempBear = findBearing_2((*p1), (*p2));
         tempBear = fmod(tempBear + 90.0, 360);

         toDraw2 = locateDestination_2((*p2)->x, (*p2)->y, (*d1), tempBear);
         toDraw2 = translateToScreen(toDraw2.x, toDraw2.y);

         glBegin(GL_LINES);
            glVertex3f(toDrawPrev2.x, toDrawPrev2.y, 0.0);
            glVertex3f(toDraw2.x, toDraw2.y, 0.0);
         glEnd();

         if(i != 0){
            glBegin(GL_LINES);
               glVertex3f(toDraw1.x, toDraw1.y, 0.0);
               glVertex3f(toDraw2.x, toDraw2.y, 0.0);
            glEnd();
         }

         toDrawPrev1 = toDraw1;
         toDrawPrev2 = toDraw2;
      }
      i++;
      d1++;
      p2++;
   }
}

void advisory::drawStdDevPathRev(){
   // Avg path list
   std::vector<Vector2d*>::iterator p1;
   std::vector<Vector2d*>::iterator p2;
   // Std deviation list
   std::vector<double>::iterator d1;

   Vector2d toDraw1, toDrawPrev1, toDraw2, toDrawPrev2, t1, t2;
   double tempBear;
   int i = 0;

   glColor4f(0.0, 1.0, 1.0, 1.0);
   glLineWidth(2.5);

   // New std deviation using distance
   d1 = stdDevDistRev.begin();
   p2 = avgPathLatLon.begin()+1;

   toDrawPrev1 = translateToScreen( (*avgPathLatLon.begin())->x, (*avgPathLatLon.begin())->y);
   toDrawPrev2 = toDrawPrev1;
   for(p1 = avgPathLatLon.begin(); p2 != avgPathLatLon.end(); p1++){
      if(i == 0 || i == 2 || i == 6 || i == 10 || i == 14 || i == 22){
         tempBear = findBearing_2((*p1), (*p2));
         tempBear = tempBear - 90.0;
         if(tempBear < 0.0){
            tempBear = tempBear + 360;
         }

         toDraw1 = locateDestination_2((*p2)->x, (*p2)->y, (*d1), tempBear);
         toDraw1 = translateToScreen(toDraw1.x, toDraw1.y);

         glBegin(GL_LINES);
            glVertex3f(toDrawPrev1.x, toDrawPrev1.y, 0.0);
            glVertex3f(toDraw1.x, toDraw1.y, 0.0);
         glEnd();

         tempBear = findBearing_2((*p1), (*p2));
         tempBear = fmod(tempBear + 90.0, 360);

         toDraw2 = locateDestination_2((*p2)->x, (*p2)->y, (*d1), tempBear);
         toDraw2 = translateToScreen(toDraw2.x, toDraw2.y);

         glBegin(GL_LINES);
            glVertex3f(toDrawPrev2.x, toDrawPrev2.y, 0.0);
            glVertex3f(toDraw2.x, toDraw2.y, 0.0);
         glEnd();

         if(i != 0){
            glBegin(GL_LINES);
               glVertex3f(toDraw1.x, toDraw1.y, 0.0);
               glVertex3f(toDraw2.x, toDraw2.y, 0.0);
            glEnd();
         }

         toDrawPrev1 = toDraw1;
         toDrawPrev2 = toDraw2;
      }
      i++;
      d1++;
      p2++;
   }
}

void advisory::drawNewAvg(){
   std::vector<Vector2d*>::iterator pos1;
   std::vector<Vector2d*>::iterator pos2;
   Vector2d draw1, draw2;
   float lineWidth;
   float tempTrans;

   // Display generated paths
   
      int test = 0;
      pos2 = newAvg.begin() + 1;
      lineWidth = 1.5;
      tempTrans = 1.0; 
      cout << "Path size: " << newAvg.size() << endl;
      for (pos1 = newAvg.begin(); pos2 != newAvg.end(); pos1++) {
         glColor4f(0.0, 0.0, 0.0, tempTrans);
         glLineWidth(lineWidth * 1.0);
         //draw1 = translateToScreen((*pos1)->x, (*pos1)->y);
         //draw2 = translateToScreen((*pos1)->x, (*pos1)->y);
         draw1.x = (*pos1)->x;
         draw1.y = (*pos1)->y;
         draw2.x = (*pos2)->x;
         draw2.y = (*pos2)->y;
         cout << "drawing: " << draw1 << " to " << draw2 << endl;
         glBegin(GL_LINES);
            glVertex3f(draw1.x, draw1.y, 0.0);
            glVertex3f(draw2.x, draw2.y, 0.0);
         glEnd();
         pos2++;
         test++;
   }
}

void advisory::drawChips(){
   std::vector<chip*>::iterator _i;

   for(_i = chips.begin(); _i != chips.end(); _i++){
      (*_i)->drawChip();
   }
}

void advisory::drawChipText(QGLWidget * g){
   Vector2d p;
   QString s;
   QFont f;

   int counter, i;

   f = QFont();
   f.setBold(true);
   glColor4f(0.0f, 0.0f, 0.0f, 1.0f);

   counter = chips.size();
   for(i = 0; i < counter; i++){
      if(chips[i]->getDrawChip() == true){
        p = chips[i]->getPos();
        s = QString::number(chips[i]->getValue());

        // Value of 20
        if(i < 2){
           f.setPointSize(20);
           g->renderText(p.x-18, p.y+10, 0, s, f, 2000);
        }
        // Value of 10
        else if(i < 6){
           f.setPointSize(15);
           g->renderText(p.x-13, p.y+7, 0, s, f, 2000);
        }
        // Value of 5
        else if(i < 9){
           f.setPointSize(13);
           g->renderText(p.x-5, p.y+7, 0, s, f, 2000);
        }
        // Value of 1
        else{
           f.setPointSize(11);
           g->renderText(p.x-4, p.y+5, 0, s, f, 2000);
        }
     }
   }
}

// Functions for sorting the paths based on their distanced at a segment to the predicted path 
void advisory::sortPath(int seg){
   sortInd = seg;

   std::sort(pathList.begin(), pathList.end(), sp);
}

// Build max and min paths using normal distribution data
void advisory::buildStatPaths(){
   Vector2d * np, * p0, p1, p2, tNP;
   double bearing, dist;
   double dInit;
   int i, j;

   //cout << "Building max stat path:   " << endl;
   // Push first point to all
   np = new Vector2d(pre->predPathThreeHour[0]->x, pre->predPathThreeHour[0]->y);
   p0 = np;
   p1.set(pre->predPathThreeHour[0]->x, pre->predPathThreeHour[0]->y);
   p2.set(pre->predPathThreeHour[1]->x, pre->predPathThreeHour[1]->y);
   //bearing = findBearing_2(&p1, &p2);
   bearing = deg;
   //dInit = haversine(pre->predPathThreeHour[0]->x, pre->predPathThreeHour[1]->x,
   //                 pre->predPathThreeHour[0]->y, pre->predPathThreeHour[1]->y)/3.0;
   dInit = speed;
   dist = dInit;
   maxPath.push_back(np);
   minPath.push_back(np);
   midPath.push_back(np);

   // Build max path
   for(i = 0; i < pre->predPathSeg.size(); i++){
      //cout << "   bearing: " << bearing;
      //cout << "   dist: " << dist << endl;
      //cout << "          adding: " << pre->predPathSeg[i]->getPMaxB();
      bearing += (pre->predPathSeg[i]->getPMaxB())/3.0;
      if(bearing < 0){
         bearing += 360.0;
      }
      bearing = fmod(bearing, 360.0);
      dist += pre->predPathSeg[i]->getSDif();
      tNP = locateDestination_2(np->x, np->y, dist*3.0, bearing);
      np = new Vector2d(tNP.x, tNP.y);
      maxPath.push_back(np);
   }

   // Build min path
   np = p0;
   //cout << endl << "Building min stat path:   " << endl;
   p1.set(pre->predPathThreeHour[0]->x, pre->predPathThreeHour[0]->y);
   p2.set(pre->predPathThreeHour[1]->x, pre->predPathThreeHour[1]->y);
   //bearing = findBearing_2(&p1, &p2);
   bearing = deg;
   dist = dInit; 
   for(i = 0; i < pre->predPathSeg.size(); i++){
      //cout << "   bearing: " << bearing;
      //cout << "   dist: " << dist;
      //cout << "          adding: " << pre->predPathSeg[i]->getPMinB() << endl;
      bearing += (pre->predPathSeg[i]->getPMinB())/3.0;
      if(bearing < 0){
         bearing += 360.0;
      }
      bearing = fmod(bearing, 360.0);
      dist += pre->predPathSeg[i]->getSDif();
      tNP = locateDestination_2(np->x, np->y, dist*3.0, bearing);
      np = new Vector2d(tNP.x, tNP.y);
      minPath.push_back(np);
   }

   // Build mid path
   np = p0;
   //cout << endl << "Building mid stat path:   " << endl;
   p1.set(pre->predPathThreeHour[0]->x, pre->predPathThreeHour[0]->y);
   p2.set(pre->predPathThreeHour[1]->x, pre->predPathThreeHour[1]->y);
   bearing = findBearing_2(&p1, &p2);
   dInit = haversine(pre->predPathThreeHour[0]->x, pre->predPathThreeHour[1]->x,
                    pre->predPathThreeHour[0]->y, pre->predPathThreeHour[1]->y)/3.0;
   dist = dInit;
   for(i = 0; i < pre->predPathSeg.size()-1; i++){
      //cout << "   bearing: " << bearing;
      //cout << "   dist: " << dist << endl;
      tNP = locateDestination_2(np->x, np->y, dist*3.0, bearing);
      np = new Vector2d(tNP.x, tNP.y);
      midPath.push_back(np);
      bearing = fmod(pre->predPathSeg[i+1]->getBDif() + bearing, 360.0);
      if(bearing < 0){
         bearing += 360.0;
      }
      dist += pre->predPathSeg[i+1]->getSDif();
   }
   tNP = locateDestination_2(np->x, np->y, dist*3.0, bearing);
   np = new Vector2d(tNP.x, tNP.y);
   midPath.push_back(np);
}

void advisory::drawMaxPath(){
   std::vector<Vector2d*>::iterator pos1;
   std::vector<Vector2d*>::iterator pos2;
   Vector2d draw1, draw2;
   float lineWidth;
   float tempTrans;
   int count = 0;

   // Display generated paths
   
      int test = 0;
      pos2 = maxPath.begin() + 1;
      lineWidth = 1.5;
      tempTrans = 1.0; 
      for (pos1 = maxPath.begin(); pos2 != maxPath.end(); pos1++) {
         if(count % 2 == 0){
            glColor4f(1.0, 0.0, 1.0, 1.0);
         }
         else{
            glColor4f(0.0, 0.0, 0.0, 1.0);
         }
         glLineWidth(2.0);
         draw1 = translateToScreen((*pos1)->x, (*pos1)->y);
         draw2 = translateToScreen((*pos2)->x, (*pos2)->y);
         glBegin(GL_LINES);
            glVertex3f(draw1.x, draw1.y, 0.0);
            glVertex3f(draw2.x, draw2.y, 0.0);
         glEnd();
         pos2++;
         test++;
         count++;
   }
}

void advisory::drawMinPath(){
   std::vector<Vector2d*>::iterator pos1;
   std::vector<Vector2d*>::iterator pos2;
   Vector2d draw1, draw2;
   float lineWidth;
   float tempTrans;
   int count = 0;

   // Display generated paths
   
      int test = 0;
      pos2 = minPath.begin() + 1;
      lineWidth = 1.5;
      tempTrans = 1.0; 
      for (pos1 = minPath.begin(); pos2 != minPath.end(); pos1++) {
         if(count % 2 == 0){
            glColor4f(0.0, 0.75, 0.75, 1.0);
         }
         else{
            glColor4f(0.0, 0.0, 0.0, 1.0);
         }
         glLineWidth(2.0);
         draw1 = translateToScreen((*pos1)->x, (*pos1)->y);
         draw2 = translateToScreen((*pos2)->x, (*pos2)->y);
         glBegin(GL_LINES);
            glVertex3f(draw1.x, draw1.y, 0.0);
            glVertex3f(draw2.x, draw2.y, 0.0);
         glEnd();
         pos2++;
         test++;
         count++;
   }
}

void advisory::drawMidPath(){
   std::vector<Vector2d*>::iterator pos1;
   std::vector<Vector2d*>::iterator pos2;
   Vector2d draw1, draw2;
   float lineWidth;
   float tempTrans;

   // Display generated paths
   
      int test = 0;
      pos2 = midPath.begin() + 1;
      lineWidth = 1.5;
      tempTrans = 1.0; 
      for (pos1 = midPath.begin(); pos2 != midPath.end(); pos1++) {
         glColor4f(0.75, 0.0, 0.75, 1.0);
         glLineWidth(2.0);
         draw1 = translateToScreen((*pos1)->x, (*pos1)->y);
         draw2 = translateToScreen((*pos2)->x, (*pos2)->y);
         glBegin(GL_LINES);
            glVertex3f(draw1.x, draw1.y, 0.0);
            glVertex3f(draw2.x, draw2.y, 0.0);
         glEnd();
         pos2++;
         test++;
   }
}

void advisory::buildECSmooth(){
   double h;
   int i;
   Vector2d p0, p1;
   double initBear;

   // First Side
   cout << "Smoothing ec1" << endl;
   for(i=1; i<errorCone1.size(); i++){
      p0 = errorCone1[i];
      p1 = errorCone1[i+1];

      h = 0.0;
      if(i==0){ h=9.0; }
      else if(i==1 || i==2 || i==3){ h=12.0; }
      else if(i==4){ h=24.0; }
      
      //interpECSmooth(p0, p1, h, 0);
      initBear = findBearing_2(&errorCone1[i-1], &errorCone1[i]);
      interpECSmooth(p0, p1, initBear, h, 0);
   }
   cout << "Finished smoothing ec1" << endl;

   // Second Side
   cout << "side 2..." << endl;
   for(i=1; i<errorCone2.size(); i++){
      p0 = errorCone2[i];
      p1 = errorCone2[i+1];

      h = 0.0;
      if(i==0){ h=9.0; }
      else if(i==1 || i==2 || i==3){ h=12.0; }
      else if(i==4){ h=24.0; }
      
      initBear = findBearing_2(&errorCone2[i-1], &errorCone2[i]);
      interpECSmooth(p0, p1, initBear, h, 1);
   }
   cout << "finished side 2..." << endl;

}

void advisory::interpECSmooth(Vector2d p0, Vector2d p1, double b, double h, int side){
   double curTime = 0.0;
   double dist = haversine(p0.x, p1.x, p0.y, p1.y);
   double speed = dist/h;
   int segments = (int)(h/3.0);
   double disPerSeg = dist/(double)segments;
   double dBear;

   Vector2d curPos = p0;
   double tBear = findBearing_2(&curPos, &p1);

   Vector4d * toAdd;
   int i;

   toAdd = new Vector4d(curPos.x, curPos.y, tBear, speed);
   if(side==0){ ecSmooth1.push_back(toAdd); }
   else{ ecSmooth2.push_back(toAdd); }

   tBear = findBearing_2(&p0, &p1);
   dBear = (findDif(tBear, b))/segments;
   tBear = b;

   for(i=0; i < segments-1; i++){
      tBear = fmod(tBear + dBear, 360.0);
      curPos = locateDestination_2(curPos.x, curPos.y, disPerSeg, tBear);

      toAdd = new Vector4d(curPos.x, curPos.y, tBear, speed);
      if(side == 0){ ecSmooth1.push_back(toAdd); }
      else{ ecSmooth2.push_back(toAdd); } 
   }

   if(h == 24){
      toAdd = new Vector4d(p1.x, p1.y, tBear, speed);
      if(side == 0){ ecSmooth1.push_back(toAdd); }
      else{ ecSmooth2.push_back(toAdd); }
   }
}


void advisory::selectChip(Vector2d p){
  std::vector<chip*>::iterator _i;
  std::vector<sector*>::iterator _j;

  int found;

  for(_i = chips.begin(); _i != chips.end(); _i++){
    (*_i)->selectChip(p);
  }

  // Test if user is grabbing a chip from a sector that is already placed
  for(_j = sectorList.begin(); _j != sectorList.end(); _j++){
    found = (*_j)->findChip(p);

    if(found > -1){
      for(_i = chips.begin(); _i != chips.end(); _i++){
        if( static_cast<int>((*_i)->getValue()) == found && (*_i)->getDrawChip() == false ){
          (*_i)->setDrawChip(true);
          (*_i)->setPos(p);
          (*_i)->selectChip(p);
          (*_i)->setSector(-1);
          break;
        }
      }
    }
  }
}

void advisory::releaseChip(){
   std::vector<chip*>::iterator _i;
   Vector2d p, sectP, pos0;
   int i, j, startPos;
   int initCS, finCS, posFill;
   double r, t, x, y;
   double angle_radians;

   cout << "Start release chip...";
   p = translateToScreen(projPath[0]->x, projPath[0]->y); 
   for(_i = chips.begin(); _i != chips.end(); _i++){
      initCS = (*_i)->getChipSect();
      finCS = (*_i)->releaseChip(p, sectorEnds, thetaList, rad);
      cout << "InitCS: " << initCS << "    finCS: " << finCS << endl;
      if(finCS == -1){
        cout << "Setting initital position..." << endl;
        (*_i)->setInitPos();
      }
      if(finCS != initCS){
         if(initCS != -1){
            //sectorFull[initCS]->at((*_i)->getSectFillPos()) = 0;
            cout << "removing chip..." << endl;
            sectorList[initCS]->removeChip(static_cast<int>((*_i)->getValue()));
            cout << "    done." << endl;
            (*_i)->setDrawChip(true);
         }
         if(finCS != -1){
            // Find correct position to start looking
            /*startPos = -1;
            if((*_i)->getValue() == 20){ startPos = 0; }
            else if((*_i)->getValue() == 10){ startPos = 2; }
            else if((*_i)->getValue() == 5){ startPos = 6; }
            else if((*_i)->getValue() == 1){ startPos = 9; }
            posFill = startPos;

            for(j = startPos; j < sectorFull[finCS]->size(); j++){
               cout << "j: " << j << endl;
               cout << flush;
               if(sectorFull[finCS]->at(j) == 0){
                  break;
               }
               else{
                  posFill++;
               }
            }
            sectorFull[finCS]->at(posFill) = 1;

            // Find lock position
            pos0 = translateToScreen(projPath[0]->x, projPath[0]->y); 
            r = chipDistList[posFill];
            t = chipThetaList[posFill]+thetaList[(*_i)->getChipSect()];

            cout << "Center position: " << pos0 << endl;
            cout << "PosFill: " << posFill << endl;
            cout << "   R: " << r << endl;
            cout << "   T: " << t << endl;
            cout << "        from: " << chipThetaList[posFill] << "   and: " << thetaList[(*_i)->getChipSect()];

            cout << "Theta list: " << endl;
            for(i = 0; i < thetaList.size(); i++){
               cout << " " << i << ": " << thetaList[i] << endl;
            }

            cout << "Chip theta list: " << endl;
            for(i = 0; i < chipThetaList.size(); i++){
               cout << " " << i << ": " << chipThetaList[i] << endl;
            }
            
            angle_radians = t*(double)M_PI/180.0;
            x = pos0.x + r*(double)sin(angle_radians);
            y = pos0.y + -1*r*(double)cos(angle_radians);
            sectP.set(x, y);

            (*_i)->setPos(sectP);
            (*_i)->setSectFillPos(posFill);*/

            sectorList[finCS]->addChip(static_cast<int>((*_i)->getValue()));
            (*_i)->setDrawChip(false);
         }
      }
   }

   // Print chip positions
   /*i = 0;
   cout << endl;
   for(_i = chips.begin(); _i != chips.end(); _i++){
      cout << i << ": ";
      //(*_i)->printPos();
   }*/
   cout << "   finished." << endl;
}

void advisory::moveChip(Vector2d v){
   std::vector<chip*>::iterator _i;

   for(_i = chips.begin(); _i != chips.end(); _i++){
      (*_i)->moveChip(v);
   }
}

// Return 1 if all chips have been placed, 0 if not
int advisory::chipsPlaced(){
   std::vector<chip*>::iterator _i;

   for(_i = chips.begin(); _i != chips.end(); _i++){
      if((*_i)->getChipSect() == -1){
         return 0;
      }
   }
   return 1;
}

void advisory::buildTargetArea(){
   int angle;
   double angle_radians;
   double x, y;
   double x1, y1;

   Vector2d pos;
   Vector2d * push;

   pos = translateToScreen(projPath[0]->x, projPath[0]->y); 

   for(angle = 0; angle < 360; angle += 5){
      angle_radians = angle*(double)M_PI/180.0;
      x = pos.x + rad*(double)cos(angle_radians);
      y = pos.y + rad*(double)sin(angle_radians);
      push = new Vector2d(x, y);
      targetArea.push_back(push);
   }
}

void advisory::buildSectors(){
   int angle, i;
   double angle_radians;
   double x, y;
   double x1, y1;
   double theta;

   Vector2d pos0, pos1, pos2, u1, u2;
   Vector2d pos;
   Vector2d * push;

   sector * s;

   pos0 = translateToScreen(projPath[0]->x, projPath[0]->y); 
   pos1 = translateToScreen(projPath[2]->x, projPath[2]->y); 
   u1 = (pos1-pos0).normalize();
   u1.set(0.0, -1.0);

   x = pos0.x + rad*u1.x;
   y = pos0.y + rad*u1.y;

   push = new Vector2d(x, y);
   sectorEnds.push_back(push);

   theta = 332.5;

   thetaList.push_back(theta);

   for(i = 0; i < 7; i++){
      //cout << "Theta init: " << theta << endl;
      theta = fmod(theta + 45.0, 360.0);
      thetaList.push_back(theta);
      //cout << "Theta: " << theta << endl;
      angle_radians = theta*(double)M_PI/180.0;
      x = pos0.x + rad*(double)sin(angle_radians);
      y = pos0.y + -1*rad*(double)cos(angle_radians);
      push = new Vector2d(x, y);
      sectorEnds.push_back(push);
   }


   pos = translateToScreen(projPath[0]->x, projPath[0]->y); 
   theta = 337.5;
   for(i = 0; i < 8; i++){
     s = new sector(theta, rad, pos);
     sectorList.push_back(s);
     theta = fmod(theta + 45.0, 360.0);
   }
   detChipLockPos();
}

void advisory::drawTargetArea(QGLWidget * g){
   std::vector<Vector2d*>::iterator _i;
   glColor4f(0.5, 0.0, 0.5, 1.0);
   glLineWidth(2);
   glBegin(GL_LINE_LOOP);
      for(_i = targetArea.begin(); _i != targetArea.end(); _i++){
         glVertex3f((*_i)->x,(*_i)->y,0.0); 
      }
   glEnd();

   drawSectors(g);
}

void advisory::drawSectors(QGLWidget * g){
   /*std::vector<Vector2d*>::iterator _i;
   Vector2d pos;

   pos = translateToScreen(projPath[0]->x, projPath[0]->y); 

   glColor4f(0.5, 0.0, 0.5, 1.0);
   glLineWidth(2);
   glBegin(GL_LINES);
      for(_i = sectorEnds.begin(); _i != sectorEnds.end(); _i++){
         glVertex3f(pos.x, pos.y, 0.0);
         glVertex3f((*_i)->x,(*_i)->y,0.0);
      }
   glEnd();*/

   std::vector<sector*>::iterator _s;
   for(_s = sectorList.begin(); _s != sectorList.end(); _s++){
     (*_s)->drawSectors(g);
   }
}

void advisory::printSectors(){
   std::vector<Vector2d*>::iterator _i;
   Vector2d pos;

   int i;
   cout << "Printing sectors..." << endl;
   pos = translateToScreen(projPath[0]->x, projPath[0]->y);

   cout << "Center: " << pos << endl;
   for(_i = sectorEnds.begin(); _i != sectorEnds.end()-6; _i++){
      cout << "   " << *(*_i) << endl;
   }

   cout << "Proj path: " << endl;
   for(i = 0; i < projPath.size(); i++){
      cout << "   " << *(projPath[i]) << endl;
   }
   cout << "Finished! " << endl;
}

void advisory::detChipLockPos(){
   std::vector<Vector2d*> chipSamplePos;
   std::vector<Vector2d*>::iterator _c;
   Vector2d * samp;
   Vector2d c, s0, s1, p, dif, u1, u2;
   double d;

   int angle, i;
   double angle_radians;
   double x, y;
   double x1, y1;
   double theta, baseTheta;

   // Sample positions from hand placement of chips
   samp = new Vector2d(647, 209);
   chipSamplePos.push_back(samp);
   samp = new Vector2d(707, 228);
   chipSamplePos.push_back(samp);
   samp = new Vector2d(755, 254);
   chipSamplePos.push_back(samp);
   samp = new Vector2d(638, 263);
   chipSamplePos.push_back(samp);
   samp = new Vector2d(681, 275);
   chipSamplePos.push_back(samp);
   samp = new Vector2d(723, 285);
   chipSamplePos.push_back(samp);
   samp = new Vector2d(632, 302);
   chipSamplePos.push_back(samp);
   samp = new Vector2d(663, 311);
   chipSamplePos.push_back(samp);
   samp = new Vector2d(695, 320);
   chipSamplePos.push_back(samp);
   samp = new Vector2d(633, 328);
   chipSamplePos.push_back(samp);
   samp = new Vector2d(654, 335);
   chipSamplePos.push_back(samp);
   samp = new Vector2d(674, 342);
   chipSamplePos.push_back(samp);
   samp = new Vector2d(637, 350);
   chipSamplePos.push_back(samp);
   samp = new Vector2d(655, 357);
   chipSamplePos.push_back(samp);

   // Center and sector edges
   c.set(614.4, 425);
   s1.set(614.4, 715);
   baseTheta = 0.0;


   //cout << "BaseTheta : " << baseTheta << endl;

   angle_radians = 0.0*(double)M_PI/180.0;
   x = c.x + rad*(double)sin(angle_radians);
   y = c.y + -1*rad*(double)cos(angle_radians);
   s0.set(x, y);
   u2 = (s0-c).normalize();
   for(_c = chipSamplePos.begin(); _c != chipSamplePos.end(); _c++){
      u1 = (*(*_c)-c).normalize();
      theta = (acos(u1*u2)*180.0)/M_PI;

      if(u1.x < u2.x){
         theta = 360.0 - theta;
      }
      else if(u1.x == u2.x){
         if(u1.y == u2.y){
            theta = 0.0;
         }
         else{
            theta = 180.0;
         }
      }

      chipThetaList.push_back(theta-baseTheta);
      dif.set(abs((*_c)->x-c.x), abs((*_c)->y-c.y));
      d = sqrt(dif.x*dif.x + dif.y*dif.y);
      chipDistList.push_back(d);
   }

}

int advisory::sectorValue(int i){
   std::vector<chip*>::iterator _c;
   int val = 0;

   for(_c = chips.begin(); _c != chips.end(); _c++){
      if( (*_c)->getChipSect() == i ){
         val += (*_c)->getValue();
      }
   }

   return val;
}
