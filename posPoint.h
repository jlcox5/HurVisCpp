/* Jonathan Cox
   Clemson University
   Position Point
*/

#include "Vector.h"

#ifndef POSPOINT_H
#define POSPOINT_H

class posPoint{
   protected:
      int month;
      int day;
      double latitude;
      double longitude;
      double degree;
      double speed;
      int wind;
      Vector2d nextPosDif;
      Vector4d point;              // Velocity X, Velocity Y

   public:
      int pathId;

      posPoint(int m, int da, double lat, double lon, double deg, double s, int w, int pid);
      int getMonth(){ return month; }
      int getDay(){ return day; }
      double getLat(){ return latitude; }
      double getLon(){ return longitude; }
      double getDeg(){ return degree; }
      double getSpeed(){ return speed; }
      int getWind(){ return wind; }
      void setDeg(double d){ degree = d; }
      void setSpeed(double s){ speed = s; }
      Vector2d * getPosDif(){ return &nextPosDif; }
      void setPosDif(Vector2d toSet){ nextPosDif = toSet; }
      Vector4d getPoint(){ return point; }
};

#endif
