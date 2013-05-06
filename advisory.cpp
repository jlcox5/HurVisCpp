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
      double curLat, double curLon, double curDeg, double curSpeed, string predFile){

   chip * c;

   // Assign position, etc
   lat = curLat;
   lon = curLon;
   deg = curDeg;
   presLat = -1;
   presLon = -1;

   // Build the projected path
   buildProjPath(projPathPos, curDeg);
   buildErrorCone();
   speed = curSpeed;
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

   usePredictedPercent = 0.0;
   baseConePercent = 0.682;

   percBubDrawn = 0.0;
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
      toAdd->w = (tempDist/time)*sim->getHours();
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
      steps += 1;
   }
}


// Chris -- The fuctions buildErrorCone(), buildECSmooth(), and interpECSmooth() should give
// you what you want. =)

// Find the points along that error cone that correspond to the points given in the NHC
// Advisory
void advisory::buildErrorCone(){
   // Radius of the error cone using 2010 values
   double errorRad[] = {50.004, 100.471, 144.302, 187.515, 283.263, 390.463};
   int i;

   // Avg path list
   std::vector<Vector4d*>::iterator p1;
   std::vector<Vector4d*>::iterator p2;
   std::vector<Vector2d> errorCone1;
   std::vector<Vector2d> errorCone2;

   Vector2d endPoint1, endPoint2;

   Vector2d tempPoint, mapPos, * t1,  * t2;
   double tempBear, startBear, endBear;

   startBear = endBear = 0.0;

   // New std deviation using distance
   t1 = new Vector2d();
   t2 = new Vector2d();

   // Build first side
   mapPos.set((*projPath.begin())->x, (*projPath.begin())->y);
   errorCone1.push_back(mapPos);
   for(p1 = projPath.begin(), p2=projPath.begin()+1, i=0; 
       p2 != projPath.end()-1; 
       p1++, p2++, i++){

      t1->set((*p1)->x, (*p1)->y);
      t2->set((*p2)->x, (*p2)->y);
      tempBear = findBearing_2(t1, t2) - 90.0;
      if(tempBear < 0.0){
         tempBear = tempBear + 360;
      }
      tempPoint = locateDestination_2(t2->x, t2->y, errorRad[i], tempBear);
      mapPos.set(tempPoint.x, tempPoint.y);
      errorCone1.push_back(mapPos);
   }

   // Build second side
   mapPos.set((*projPath.begin())->x, (*projPath.begin())->y);
   errorCone2.push_back(mapPos);
   for(p1 = projPath.begin(), p2=projPath.begin()+1, i=0; 
       p2 != projPath.end()-1;
       p1++, p2++, i++){

      t1->set((*p1)->x, (*p1)->y);
      t2->set((*p2)->x, (*p2)->y);
      tempBear = fmod(findBearing_2(t1, t2) + 90.0, 360.0);
      tempPoint = locateDestination_2(t2->x, t2->y, errorRad[i], tempBear);
      mapPos.set(tempPoint.x, tempPoint.y);
      errorCone2.push_back(mapPos);
   }

   // Interpolate to find the points inbetween the error cone points that correspond
   // to the advisory positions.
   buildECSmooth(errorCone1, errorCone2);

   // Build the display for the NHC error cone in the user study
   buildTrueCone();

   delete(t1);
   delete(t2);
}

void advisory::buildECSmooth(std::vector<Vector2d> & errorCone1, 
                             std::vector<Vector2d> & errorCone2){
   double h;
   unsigned int i;
   Vector2d p0, p1;

   // First Side
   for(i=0; i<errorCone1.size(); i++){
      p0 = errorCone1[i];
      p1 = errorCone1[i+1];

      h = 0.0;
      if(i==0){ h=9.0; }
      else if(i==1 || i==2 || i==3){ h=12.0; }
      else if(i==4){ h=24.0; }
      
      interpECSmooth(p0, p1, h, 0);
   }

   // Second Side
   for(i=0; i<errorCone2.size(); i++){
      p0 = errorCone2[i];
      p1 = errorCone2[i+1];

      h = 0.0;
      if(i==0){ h=9.0; }
      else if(i==1 || i==2 || i==3){ h=12.0; }
      else if(i==4){ h=24.0; }
      
      interpECSmooth(p0, p1, h, 1);
   }
}

// Interpolate between p0 and p1 to find additional points on the error cone
void advisory::interpECSmooth(Vector2d p0, Vector2d p1, double h, int side){
   double dist = haversine(p0.x, p1.x, p0.y, p1.y);
   double speed = dist/h;
   int segments = (int)(h/3.0);
   double disPerSeg = dist/(double)segments;

   Vector2d curPos = p0;
   double tBear = findBearing_2(&curPos, &p1);

   Vector4d * toAdd;
   int i;

   toAdd = new Vector4d(curPos.x, curPos.y, tBear, speed);
   if(side==0){ ecSmooth1.push_back(toAdd); }
   else{ ecSmooth2.push_back(toAdd); }

   for(i=0; i < segments-1; i++){
      curPos = locateDestination_2(curPos.x, curPos.y, disPerSeg, tBear);
      tBear = findBearing_2(&curPos, &p1);

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

// Chris --- Ignore buildTrueCone() and buildTrueConeSide()
void advisory::buildTrueCone(){

  buildTrueConeSide(0);
  buildTrueConeSide(1);

  cout << "Finished building the true cone..." << endl;
}

void advisory::buildTrueConeSide(int side){
  // Radius of the error cone using 2010 values
  double errorRad[] = {0.0, 50.004, 100.471, 144.302, 187.515, 283.263, 390.463};

  Vector2d * push;
  Vector2d p0, p1, pT;
  double l, r, R, x, d, theta, phi1, phi2;
  double bearing;

  for(unsigned int i=0; i<projPath.size()-2; i++){
    p0.set(projPath[i]->x, projPath[i]->y);
    p1.set(projPath[i+1]->x, projPath[i+1]->y);

    d = haversine(p0.x, p1.x, p0.y, p1.y);
    r = errorRad[i];
    R = errorRad[i+1];
    x = (r*d)/(R-r);
    l = sqrt((x+d)*(x+d) - R*R);
    theta = (atan2(R, l)*180.0)/M_PI;
    if(theta != theta){ theta = 0.0; }
    phi1 = (theta+90.0);
    phi2 = 180.0 - phi1;

    if(side == 0){
      bearing = findBearing_2(&p0, &p1);
      phi1 = fmod(bearing + phi1, 360.0);
      bearing = fmod(findBearing_2(&p1, &p0)+180.0, 360.0);
      phi2 = fmod(bearing + phi2, 360.0);
    }
    else{
      bearing = findBearing_2(&p0, &p1);
      phi1 = bearing - phi1;
      if(phi1 < 0.0){ phi1 += 360.0; }
      bearing = fmod(findBearing_2(&p1, &p0)+180.0, 360.0);
      phi2 = bearing - phi2;
      if(phi2 < 0.0){ phi2 += 360.0; }
    }

    // Determine points and push to the error cone
    if(side == 0){
      pT = locateDestination_2(p0.x, p0.y, r, phi1); 
      push = new Vector2d(pT.x, pT.y);
      trueCone1.push_back(push);
      pT = locateDestination_2(p1.x, p1.y, R, phi1); 
      push = new Vector2d(pT.x, pT.y);
      trueCone1.push_back(push);
    }
    else{
      pT = locateDestination_2(p0.x, p0.y, r, phi1); 
      push = new Vector2d(pT.x, pT.y);
      trueCone2.push_back(push);
      pT = locateDestination_2(p1.x, p1.y, R, phi1); 
      push = new Vector2d(pT.x, pT.y);
      trueCone2.push_back(push);
    } 
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

void advisory::drawGenPathsRainDrop(){
   std::vector<Vector2d*>::iterator pos;
   std::vector<path*>::iterator pathIt;
   Vector2d draw1, draw2;
   float tempTrans;
   float age;

   Vector2d * curPos;

   // Display generated paths
   for (pathIt = pathList.begin(); pathIt != pathList.end(); pathIt++){
      //tempTrans = (*pathIt)->getInt().w*sim->getPathOpacity();
      tempTrans = (*pathIt)->getInt().w;
      age = (*pathIt)->getAge();
      //cout << "age: " << age << endl << flush;
      float redVal = 0.5;
      float greenVal = 0.0;
      float blueVal = 0.0;
      float distAdd = 0.0;
      int i = 0;
      i = min(slider.getCurTick(), (int)((*pathIt)->posList.size())-1);
      i = max(i, 0);

      curPos = (*pathIt)->posList[i];
      //for (pos = (*pathIt)->posList.begin(); pos != (*pathIt)->posList.end(); pos++) {
        //if(i == 0 || i ==3 || i == 7 || i == 11 || i == 15 || i == 23){

          int angle;
          double angle_radians;
          double x, y;
          double x1, y1;

          float newTempTrans = tempTrans - (distAdd/1.5);
          float newDropRadius = 3 + 7*(1.0 - newTempTrans);
          glColor4f(redVal, (greenVal+(i*0.02)), blueVal+(i*0.02), newTempTrans);

          //if(i == 3){
          //  glColor4f(0.0, 0.0, 1.0, 1.0);
          //}

          x1 = y1 = 0.0;
  
          glLineWidth(2.0);
          //glBegin(GL_LINE_LOOP);
          glBegin(GL_POLYGON);
          for(angle = 0; angle < 360; angle += 5){
            angle_radians = angle*(double)M_PI/180.0;
            //x = (*pos)->x + newDropRadius*(double)cos(angle_radians);
            //y = (*pos)->y + newDropRadius*(double)sin(angle_radians);
            x = curPos->x + newDropRadius*(double)cos(angle_radians);
            y = curPos->y + newDropRadius*(double)sin(angle_radians);
            if(angle == 0){
              x1 = x; 
              y1 = y;
            }
            glVertex3f(x,y,0.0); 
          }
          glVertex3f(x1,y1,0.0); 
          glEnd();

          /*newDropRadius = newDropRadius/5;
          glBegin(GL_POLYGON);
          for(angle = 0; angle < 360; angle += 5){
            angle_radians = angle*(double)M_PI/180.0;
            x = (*pos)->x + newDropRadius*(double)cos(angle_radians);
            y = (*pos)->y + newDropRadius*(double)sin(angle_radians);
            if(angle == 0){
              x1 = x; 
              y1 = y;
            }
            glVertex3f(x,y,0.0); 
          }
          glVertex3f(x1,y1,0.0); 
          glEnd();*/

          glDisable(GL_DEPTH_TEST);
          distAdd += 0.02;

          greenVal += 0.02;
          blueVal += 0.02; 

        //  i++;
        //  if(i > slider.getCurTick()){
        //    break;
        //  }
        //}

        (*pathIt)->setInt((*pathIt)->getInt().w - ((age)*0.000005));
        (*pathIt)->incAge();
      //}
   }
   for (pathIt = pathList.begin(); pathIt != pathList.end();){
     if((*pathIt)->getInt().w <= 0.0){
       delete((*pathIt));
       pathIt = pathList.erase(pathIt);
     }
     else{
       pathIt++;
     }
   }
}

void advisory::drawGenPathsEyePoint(){
  std::vector<hurEyePoints*>::iterator _e;

  for(_e = eyePointList.begin(); _e != eyePointList.end(); _e++){
    (*_e)->drawPoints();
    (*_e)->age(0.01);
    if( (*_e)->getTrans() < 0.0 ){
      delete((*_e));
      eyePointList.erase(_e);
    }
  }
}

int advisory::drawGenPathsTrail(int step){
  std::vector<path*>::iterator pathIt;
  Vector2d * p1;
  Vector2d * p2;
  double trans = 1.0;
  bool drawingRad = false;

  if(step > 23){ return step; }

  for(pathIt = pathList.begin(); pathIt != pathList.end(); pathIt++){
    //cout << "Step: " << step << endl << flush;
    trans = 1.0;
    for(int curStep = step-1; curStep >= 0; curStep--){
      p1 = (*pathIt)->posList[curStep]; 
      p2 = (*pathIt)->posList[curStep+1]; 
      glLineWidth(1.0);
      //glColor4f(0.0, 0.0, 1.0, trans);
      glColor4f(0.0, 0.0, 1.0, 0.5);

      glBegin(GL_LINES);
        glVertex3f(p1->x, p1->y, 0.0);
        glVertex3f(p2->x, p2->y, 0.0);
      glEnd();
      trans -= 0.1;

      if((step == 3 || step == 7 || step == 11 || step == 15 || step == 23)
          && (curStep == step - 1)){
        drawingRad = true;
        if(percBubDrawn < 1.0){
          int angle;
          double angle_radians;
          double x, y;
          double x1 = 0.0;
          double y1 = 0.0;
  
          float newTempTrans = 1.0 - (0.5*percBubDrawn);
          float newDropRadius = 3 + (4*percBubDrawn);
          glColor4f(0.0, 0.0, 1.0, newTempTrans);
  
          glLineWidth(2.0);
          //glBegin(GL_LINE_LOOP);
          glBegin(GL_POLYGON);
          for(angle = 0; angle < 360; angle += 5){
            angle_radians = angle*(double)M_PI/180.0;
            x = p2->x + newDropRadius*(double)cos(angle_radians);
            y = p2->y + newDropRadius*(double)sin(angle_radians);
            if(angle == 0){
              x1 = x; 
              y1 = y;
            }
            glVertex3f(x,y,0.0); 
          }
          glVertex3f(x1,y1,0.0); 
          glEnd();
        }
      }
    }
  }

  if(percBubDrawn >= 1.0){
    percBubDrawn = 0.0;
    drawingRad = false;
  }
  if(drawingRad == true){
    percBubDrawn += 0.1;
    step--;
    usleep(175000);
  }
  else{
    usleep(150000);
  }

  return step;
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
         glBegin(GL_LINES);
         glVertex3f(draw1.x, draw1.y, 0.0);
         glVertex3f(draw2.x, draw2.y, 0.0);
         glEnd();
         pos2++;
         test++;
      }
   }
}

// Draw the projected path for each advisory in the simulation
void advisory::drawForecastPath() {
   Vector2d draw1, draw2;
   std::vector<Vector4d*>::iterator pos1;
   std::vector<Vector4d*>::iterator pos2;

   drawTrueCone();

   glColor4f(1.0, 1.0, 1.0, 1.0);
   glLineWidth(2.0);

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
   //drawErrorCone();
}

void advisory::drawErrorSmooth(){
   Vector2d draw1, draw2;
   std::vector<Vector4d*>::iterator pos1_4d;
   std::vector<Vector4d*>::iterator pos2_4d;


   // Draw the smoothed side
   glColor4f(0.0, 1.0, 0.0, 0.5);
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

   // Draw the smoothed side
   glColor4f(0.0, 1.0, 0.0, 0.5);
   pos2_4d = ecSmooth2.begin()+1;
   for(pos1_4d = ecSmooth2.begin(); pos2_4d != ecSmooth2.end(); pos1_4d++){
      draw1 = translateToScreen((*pos1_4d)->x, (*pos1_4d)->y);
      draw2 = translateToScreen((*pos2_4d)->x, (*pos2_4d)->y);
      glBegin(GL_LINES);
      glVertex3f(draw1.x, draw1.y, 0.0);
      glVertex3f(draw2.x, draw2.y, 0.0);
      glEnd();
      pos2_4d++;
   }
}

void advisory::drawErrorConeRadius(){
  // Draw the circles
  glColor4f(1.0, 0.0, 0.0, 1.0); 
  double errorRad[] = {0.0, 50.004, 100.471, 144.302, 187.515, 283.263, 390.463};
  double curDeg = 0;
  Vector2d draw1;

  for(unsigned int i = 1; i < projPath.size()-1; i++){
    glBegin(GL_LINE_LOOP);
    for(curDeg = 0; curDeg < 360.0; curDeg++){
      draw1 = locateDestination_2(projPath[i]->x, projPath[i]->y, errorRad[i], curDeg);
      draw1 = translateToScreen(draw1.x, draw1.y);
      glVertex3f(draw1.x, draw1.y, 0.0);
    }
    glEnd();
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

void advisory::drawChipBox(){
  double bx, by, tx, ty;

  bx = 815.0;
  by = 7.0;
  tx = 1017.0;
  ty = 183.0;

  glColor4f(0.0, 0.0, 0.0, 0.9);
  glLineWidth(3.0);
  glBegin(GL_LINES);
   glVertex3f(bx, by, 0.0); 
   glVertex3f(tx, by, 0.0); 

   glVertex3f(tx, ty, 0.0); 
   glVertex3f(bx, ty, 0.0); 

   glVertex3f(bx, ty, 0.0); 
   glVertex3f(bx, by, 0.0); 

   glVertex3f(tx, by, 0.0); 
   glVertex3f(tx, ty, 0.0); 
  glEnd();

  glColor4f(0.5, 0.5, 0.5, 0.9);
  glBegin(GL_POLYGON);
   glVertex3f(bx, by, 0.0); 
   glVertex3f(tx, by, 0.0); 
   glVertex3f(tx, ty, 0.0); 
   glVertex3f(bx, ty, 0.0); 
  glEnd();
}

void advisory::drawSlider(){
  slider.drawSlider();
}

// Functions for sorting the paths based on their distanced at a segment to the predicted path 
void advisory::sortPath(int seg){
   sortInd = seg;

   std::sort(pathList.begin(), pathList.end(), sp);
}

void advisory::drawTrueCone(){
  Vector2d draw1, draw2, draw3, draw4;
  std::vector<Vector2d*>::iterator _p1;
  std::vector<Vector2d*>::iterator _p2;
  std::vector<Vector2d*>::iterator _p3;
  std::vector<Vector2d*>::iterator _p4;
  

  glColor4f(0.5, 0.5, 1.0, 0.5);
  glLineWidth(1.5);

  //glEnable(GL_DEPTH_TEST);
  /*glBlendColor(0.0, 0.0, 1.0, 0.5);
  glEnable(GL_COLOR_LOGIC_OP);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  _p3 = trueCone2.begin();
  for(_p1 = trueCone1.begin(); _p1 != trueCone1.end()-1; _p1++){
    _p2 = _p1 + 1;
    _p4 = _p3 + 1;
    draw1 = translateToScreen((*_p1)->x, (*_p1)->y);  
    draw2 = translateToScreen((*_p2)->x, (*_p2)->y);
    draw3 = translateToScreen((*_p3)->x, (*_p3)->y);  
    draw4 = translateToScreen((*_p4)->x, (*_p4)->y);
    glLogicOp(GL_AND);
    glBegin(GL_POLYGON);
      glVertex3f(draw1.x, draw1.y, 0.0);
      glVertex3f(draw2.x, draw2.y, 0.0);
      glVertex3f(draw4.x, draw4.y, 0.0);
      glVertex3f(draw3.x, draw3.y, 0.0);
    glEnd();
    count++;
    _p3++;
  }*/
  
  // Draw the circles
  glColor4f(0.5, 0.5, 1.0, 0.5); 
  double errorRad[] = {0.0, 50.004, 100.471, 144.302, 187.515, 283.263, 390.463};
  double curDeg = 0;
  for(unsigned int i = 1; i < projPath.size()-1; i++){
    glLogicOp(GL_AND);
    glBegin(GL_POLYGON);
    for(curDeg = 0; curDeg < 360.0; curDeg++){
      draw1 = locateDestination_2(projPath[i]->x, projPath[i]->y, errorRad[i], curDeg);
      draw1 = translateToScreen(draw1.x, draw1.y);
      glVertex3f(draw1.x, draw1.y, 0.0);
    }
    glEnd();
  }

  glDisable(GL_DEPTH_TEST);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glDisable(GL_COLOR_LOGIC_OP);
}

void advisory::selectChip(Vector2d p){
  std::vector<chip*>::iterator _i;
  std::vector<sector*>::iterator _j;
  int sectorFound;

  int found;

  // Test for grabbing visible chip
  for(_i = chips.begin(); _i != chips.end(); _i++){
    (*_i)->selectChip(p);
  }

  // Test if user is grabbing a chip from a sector that is already placed
  sectorFound = 0;
  for(_j = sectorList.begin(); _j != sectorList.end(); _j++){
    found = (*_j)->findChip(p);

    cout << "found: " << found << endl;

    if(found > -1){
      for(_i = chips.begin(); _i != chips.end(); _i++){
        if( static_cast<int>((*_i)->getValue()) == found && (*_i)->getDrawChip() == false &&
            (*_i)->getChipSect() == sectorFound){
          (*_i)->setDrawChip(true);
          (*_i)->setPos(p);
          (*_i)->selectChip(p);
          (*_i)->setSector(-1);
          break;
        }
      }
    }
    sectorFound++;
  }
}

void advisory::releaseChip(){
   std::vector<chip*>::iterator _i;
   Vector2d p, sectP, pos0;
   int initCS, finCS;

   cout << "Start release chip...";
   p = translateToScreen(projPath[0]->x, projPath[0]->y); 
   for(_i = chips.begin(); _i != chips.end(); _i++){
      initCS = (*_i)->getChipSect();
      finCS = (*_i)->releaseChip(p, thetaList, rad);
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
            sectorList[finCS]->addChip(static_cast<int>((*_i)->getValue()));
            //(*_i)->setSector(-1);
            (*_i)->setDrawChip(false);
         }
      }
   }

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
      //cout << "chips placed: " << (*_i)->getChipSect() << endl;
      if((*_i)->getChipSect() == -1){
         //cout << "returning 0" << endl;
         return 0;
      }
   }
   //cout << "returning 1" << endl;
   return 1;
}

void advisory::checkSliderPressed(Vector2d p){
   slider.checkSliderPressed(p);
}

void advisory::moveSlider(Vector2d p){
   slider.moveSlider(p);
}


void advisory::buildTargetArea(){
   int angle;
   double angle_radians;
   double x, y;

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
   int i;
   double angle_radians;
   double x, y;
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
   std::vector<sector*>::iterator _s;
   for(_s = sectorList.begin(); _s != sectorList.end(); _s++){
     (*_s)->drawSectors(g);
   }
}

void advisory::printSectors(){
   std::vector<Vector2d*>::iterator _i;
   Vector2d pos;

   cout << "Printing sectors..." << endl;
   pos = translateToScreen(projPath[0]->x, projPath[0]->y);

   cout << "Center: " << pos << endl;
   for(_i = sectorEnds.begin(); _i != sectorEnds.end()-6; _i++){
      cout << "   " << *(*_i) << endl;
   }

   cout << "Proj path: " << endl;
   for(unsigned int i = 0; i < projPath.size(); i++){
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

   double angle_radians;
   double x, y;
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

void advisory::findInConePercent(){
  inConePercent = 0;
  int size = pathList.size();
  int t = 0;

  if(size > 0){
    for(unsigned int i = 0; i < pathList.size(); i++){
      if(pathList[i]->getInCone() == true){
        t++;
      }
    }
    //cout << "Percentage of paths in cone: ";
    //cout << static_cast<double>(t)/static_cast<float>(size) << endl;
    //cout << "    t: " << t << "     total: " << pathList.size() << endl;
    inConePercent = static_cast<double>(t)/static_cast<float>(size);
    //cout << inConePercent << endl << flush;
  }
  updatePredictedPercent();
}

void advisory::updatePredictedPercent(){
  //cout << "inConePercent: " << inConePercent << "    baseConePercent: " << baseConePercent << endl;
  if(inConePercent < baseConePercent && usePredictedPercent < 0.9995){
    usePredictedPercent += 0.0005; 
  }
  else if(inConePercent > baseConePercent && usePredictedPercent > 0.0005){
    usePredictedPercent -= 0.0005; 
  }
  //cout << "   New use predicted percent: " << usePredictedPercent << endl;
}
