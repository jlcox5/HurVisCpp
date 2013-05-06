/* Jonathan Cox
   Clemson University
   Path
*/

#include <math.h>
#include <limits.h>
#include "path.h"
#include "advisory.h"
#include "hurVis.h"
#include "simulation.h"

extern simulation * sim;
extern int wWidth, wHeight;

extern int interactive, hitButton;

extern double curDistance;

extern std::vector<Vector2d*> segmentVelocity;

extern path * curPath;

extern double speedRatio;
extern double bearRatio;

// For random number generation
extern unsigned int counter;

path::path(double curLat, double curLon, double curDeg, double curSpeed, unsigned int curAdv){
   Vector2d * toAdd; // List of screen coords to draw the line
   Vector2d temp;
   std::vector<Vector2d*>::iterator i;
   Vector4d toSearch;  // Vector that contains the information to search for in the kdtree
   Vector4d tempForDist;
   posPoint * toPush;
   double earthRad;

   std::vector<Vector4d*> result;

   earthRad = 6378.1;
   pathFound = NULL;

   age = 0.0;
   
   double chosen = 0.0;
   double cont = 69.0/sim->getHours(); // 69 hours is time frame

   int r;
   
   curDistance = 0.0;
   dist = 0.0;
   inCone = false;
   fromHist = false;

   // For random number generation
   //unsigned int count = time(NULL);
   
   // Starting line color
   intensity.x = 0.0;
   intensity.y = 0.0;
   intensity.z = 1.0;
   intensity.w = 1.0;

   side = 0;
   
   toAdd = new Vector2d;

   toSearch.x = curLon;
   toSearch.y = curLat;
   toSearch.z = curSpeed;
   toSearch.w = curDeg;

   // Figure out position of starting point on screen and insert into list
   toAdd->x = curLon;
   toAdd->y = curLat;
   i = latLonList.end();
   latLonList.insert(i, toAdd);
   temp = translateToScreen(curLon, curLat);
   toAdd = new Vector2d;
   toAdd->x = temp.x;
   toAdd->y = temp.y;
   i = posList.end();
   posList.insert(i, toAdd);


   // Determine whether to do a historical path or a predicted path
   srand ( counter );
   counter = (counter*21)%UINT_MAX;

   r = rand() % 10000000+1;
   chosen = (double)r/10000000.0;

   //if(sim->adv->getInConePercent() <= sim->adv->getBasePercent()){
   if(chosen <= 0.68){
     speedRatio = 0.0;
     bearRatio = 0.0;
   }
   else{
     speedRatio = 0.95;
     bearRatio = 0.95;
     fromHist = true;
   }



   toPush = new posPoint(-1, -1, curLat, curLon, curDeg, curSpeed, -1, -1);
   positionList.push_back(toPush);
   //cout << "Pos 1: " << toAdd << "\n";

   curPath = NULL;
   // Translate to screen coords and insert that point into the list
   if(interactive == -1){
      /*srand ( counter );
      counter = (counter*21)%UINT_MAX;

      r = rand() % 10000000+1;
      chosen = (double)r/10000000.0;

      if(chosen < 0.35){ histOrPre = 1; }*/


      lastPosInfo.set(-1.0, -1.0, -1.0, -1.0);
      while(chosen < cont){
         //if(toSearch.x != toSearch.x){
            //cout << "Searching with: " << toSearch << "\n";
         //}

         temp.x = toSearch.x;
         temp.y = toSearch.y;

         toSearch = nextPoint(toSearch, chosen);

         curDistance = curDistance + haversine(toSearch.x, temp.x, toSearch.y, temp.y);
         toAdd = new Vector2d;
         toAdd->x = toSearch.x;
         toAdd->y = toSearch.y;
         i = latLonList.end();
         latLonList.insert(i, toAdd);

         curSpeed = (int)toSearch.z;
         curDeg = toSearch.w;
         toPush = new posPoint(-1, -1, toSearch.y, toSearch.x, curDeg, curSpeed, -1, -1);
         positionList.push_back(toPush);
         if(toSearch.x != toSearch.x){
            cout << "Lat Lon " << chosen << ": " << toSearch << "\n";
         }
         temp = translateToScreen(toSearch.x, toSearch.y);
         toAdd = new Vector2d;
         toAdd->x = temp.x;
         toAdd->y = temp.y;

         i = posList.end();
         posList.insert(i, toAdd);
         if(toAdd->x != toAdd->x || toAdd->y != toAdd->y){
            cout << "Pos " << chosen << ": " << toAdd->x << ", " << toAdd->y << "\n";
            cout << "     from point: " << toSearch << endl;
            exit(1);
         }
         chosen =  chosen + 1.0;
      }

      //smoothPath();
   }
   // If in interactive mode, just find one more line segment
   else{
      temp.x = toSearch.x;
      temp.y = toSearch.y;
      toSearch = nextPoint(toSearch, chosen);
      toAdd = new Vector2d;
      toAdd->x = toSearch.x;
      toAdd->y = toSearch.y;
      i = latLonList.end();
      latLonList.insert(i, toAdd);
      curDistance = curDistance + haversine(toSearch.x, temp.x, toSearch.y, temp.y);
      lastPosInfo.set(toSearch.x, toSearch.y, toSearch.z, toSearch.w);
      temp = translateToScreen(toSearch.x, toSearch.y);
      toAdd = new Vector2d;
      toAdd->x = temp.x;
      toAdd->y = temp.y;
   
      i = posList.end();
      posList.insert(i, toAdd);
      chosen =  chosen + 1.0;
      lastChosen = chosen;
   }
   
   dist = curDistance;

   testAgainstCone();
   //cout << "created dist: " << dist << "\n";
   //cout << "posList size: " << posList.size() << "\n";
}

path::path(double curLat, double curLon, double curDeg, double curSpeed){
   Vector2d * toAdd; // List of screen coords to draw the line
   Vector2d temp;
   std::vector<Vector2d*>::iterator i;
   Vector4d toSearch;  // Vector that contains the information to search for in the kdtree
   double earthRad;

   earthRad = 6378.1;

   // Starting line color
   intensity.x = 0.0;
   intensity.y = 0.0;
   intensity.z = 1.0;
   intensity.w = 1.0;

   toAdd = new Vector2d;

   toSearch.x = curLon;
   toSearch.y = curLat;
   toSearch.z = sin(curDeg*M_PI/180)*curSpeed*sim->getHours();
   toSearch.w = cos(curDeg*M_PI/180)*curSpeed*sim->getHours();

   // Figure out position of starting point on screen and insert into list
   temp = translateToScreen(curLon, curLat);
   toAdd->x = temp.x;
   toAdd->y = temp.y;
   i = posList.end();
   posList.insert(i, toAdd);
}

path::path(std::vector<Vector2d*> pList){
   std::vector<Vector2d*>::iterator _p;
   Vector2d * toAdd;
   Vector2d temp;;

   for(_p = pList.begin(); _p != pList.end(); _p++){
      toAdd = new Vector2d;
      toAdd->x = (*_p)->x;
      toAdd->y = (*_p)->y;
      latLonList.push_back(toAdd);
   }

   for(_p = latLonList.begin(); _p != latLonList.end(); _p++){
      toAdd = new Vector2d;
      temp = translateToScreen((*_p)->x, (*_p)->y);
      toAdd->x = temp.x;
      toAdd->y = temp.y;
      posList.push_back(toAdd);
   }

   // Starting line color
   intensity.x = 0.0;
   intensity.y = 0.0;
   intensity.z = 1.0;
   intensity.w = 1.0;

}

void path::setLastPosInfo(double lat, double lon, double velX, double velY){
   lastPosInfo.set(lon, lat, velX, velY);
}

void path::addSegment(){
   Vector4d toSearch;
   Vector2d temp;
   Vector2d * toAdd;
   std::vector<Vector2d*>::iterator i;

   toSearch = nextPoint(lastPosInfo, lastChosen);
   curDistance = curDistance + haversine(lastPosInfo.x, toSearch.x, lastPosInfo.y, toSearch.y);
   lastPosInfo.set(toSearch.x, toSearch.y, toSearch.z, toSearch.w);
   temp = translateToScreen(toSearch.x, toSearch.y);
   toAdd = new Vector2d;
   toAdd->x = temp.x;
   toAdd->y = temp.y;

   i = posList.end();
   posList.insert(i, toAdd);
   lastChosen =  lastChosen + 1.0;

   //cout << "curDistance: " << curDistance << "\n";

   dist = curDistance;
}

void path::smoothPath(){
   Vector4d * pos1, * pos2;
   double ts, velX, velY;
   int segCount;
   posPoint * toInsert;
   std::vector<Vector4d*> result;
   Vector2d * newItem, draw;

   std::vector<posPoint*>::iterator _p1;
   std::vector<posPoint*>::iterator _p2;
   std::vector<Vector4d*>::iterator _r;

   ts = 3;
   segCount = 5;

   _p2 = positionList.begin()+1;

   for(_p1 = positionList.begin(); _p2 != positionList.end(); _p2++){
      positionListAug.push_back((*_p1));
      velX = (sin((*_p1)->getDeg()*M_PI/180)*(*_p1)->getSpeed())/ts;
      velY = (cos((*_p1)->getDeg()*M_PI/180)*(*_p1)->getSpeed())/ts;
      pos1 = new Vector4d((*_p1)->getLon(), (*_p1)->getLat(), velX, velY);
      velX = (sin((*_p2)->getDeg()*M_PI/180)*(*_p2)->getSpeed())/ts;
      velY = (cos((*_p2)->getDeg()*M_PI/180)*(*_p2)->getSpeed())/ts;
      pos2 = new Vector4d((*_p2)->getLon(), (*_p2)->getLat(), velX, velY);
      result = smoothCurve(pos1, pos2, (*_p1)->getDeg(), (*_p2)->getDeg(), ts, segCount);


      for(_r = result.begin(); _r != result.end(); _r++){
         toInsert = new posPoint(1, 1, (*_r)->y, (*_r)->x, (*_r)->z, (int)(*_r)->w, 1, 1);
         positionListAug.push_back(toInsert);
      }
      positionListAug.push_back((*_p2));
      _p1++;
   }

   // Recalculate the drawing points and all
   for(_p1 = positionListAug.begin(); _p1 != positionListAug.end(); _p1++){
      draw = translateToScreen((*_p1)->getLon(), (*_p1)->getLat());
      //cout << "Lon/Lat: " << (*_p1)->getLon() << "," << (*_p1)->getLat() << " gives: " << draw << "\n";
      newItem = new Vector2d(draw.x, draw.y);
      posList2.push_back(newItem);
   }
}

void path::testAgainstCone(){
  int checkPoint[] = {3, 7, 11, 15, 23};
  double errorRad[] = {50.004, 100.471, 144.302, 187.515, 283.263};
  
  std::vector<Vector4d*>& pred = sim->adv->pre->predPathThreeHour;

  inCone = true;
  
  for(int i = 0; i < 5; i++){
    if(haversine(positionList[checkPoint[i]]->getLon(), pred[checkPoint[i]]->x, 
                 positionList[checkPoint[i]]->getLat(), pred[checkPoint[i]]->y) > errorRad[i]){
      inCone = false;
      break;
    }
  }
}

