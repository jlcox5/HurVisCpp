/* Jonathan Cox
   Clemson University
   Grid Data Point
*/

#ifndef GRIDPOINT_H
#define GRIDPOINT_H

#include <iostream>

class gridPoint{
   protected:
      float bearingDif;
      float speedDif;
      float weight;

      float startLat;
      float startLon;
      float finalLat;
      float finalLon;

   public:
      gridPoint(float b, float s, float w, float sla, float slo, float fla, float flo): bearingDif(b), speedDif(s), weight(w), startLat(sla), startLon(slo), finalLat(fla), finalLon(flo){}

      //get
      float getBD(){ return bearingDif; }
      float getSD(){ return speedDif; }
      float getW(){ return weight; }

      float getSLat(){ return startLat; }
      float getSLon(){ return startLon; }
      float getFLat(){ return finalLat; }
      float getFLon(){ return finalLon; }

      void printData(){  std::cout << bearingDif << " " << speedDif << " " << weight << " " << startLat << " " << startLon << "\n"; }
};
#endif
