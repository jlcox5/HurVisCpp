/* Jonathan Cox
   Clemson University
   Hurricane Visualization Functions
*/

#include <vector>
#include "Vector.h"
#include "posPoint.h"
#include "hurVis.h"
#include "geoFunct.h"

#ifndef PATH_H
#define PATH_H


class path{
   protected:
      Vector4d intensity;
      Vector4d lastPosInfo;
      double lastChosen;
      double dist;
      double age;
      int side;

      bool inCone;
      bool fromHist;
      
   public:
      // Contains only generated points
      std::vector<Vector2d*> posList;
      // Contains generated and smoothed points
      std::vector<Vector2d*> posList2;
      std::vector<Vector2d*> latLonList;
      // Contains only generated points
      std::vector<posPoint*> positionList;
      // Contains generated and smoothed points
      std::vector<posPoint*> positionListAug;
      path * pathFound;

      // Builds a complete path with kdTree
      path(double curLon, double curLat, double curDeg, double curSpeed, unsigned int curAdv);
      // Only inserts first point
      path(double curLon, double curLat, double curDeg, double curSpeed);
      // Already determined
      path(std::vector<Vector2d*> pList);

      std::vector<Vector2d*> getPosList(){ return posList; }
      Vector4d getInt(){ return intensity; }
      void setInt(double n){ intensity.w = n; }
      void incAge(){ age += 0.1; }
      float getAge(){ return age; }
      void setLastPosInfo(double lon, double lat, double velX, double velY);
      Vector4d getLastPosInfo(){ return lastPosInfo; }
      void addSegment();
      double getDist(){ return dist; }
      int size(){ return posList.size(); }

      void setSide(int i){ side = i; }
      int getSide(){ return side; }

      void smoothPath();

      void testAgainstCone();
      bool getInCone(){ return inCone; }

};
      
#endif
