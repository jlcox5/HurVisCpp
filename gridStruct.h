/* Jonathan Cox
   Clemson University
   Grid Structure
*/

#include <vector>
#include "gridPoint.h"
#include "bin.h"
#include "geoFunct.h"

#ifndef GRIDSTRUCT_H
#define GRIDSTRUCT_H

class gridStruct{
   protected:
      int degLat;
      int degLon;
      int degPerBin;
      int minLat;
      int maxLat;
      int minLon;
      int maxLon;

      int extraLines;   // How many additional lines to add on either side 

   public:
      std::vector<std::vector<std::vector<bin*> *> *> grid; 

      gridStruct(int dLat, int dLon, int dpBin, int miLa, int maLa, int miLo, int maLo, string filename);

      void loadFile(string filename, int t);
      void addPoints(gridPoint * g);
      void printGrid();

      int getDegPerBin(){ return degPerBin; }
      int getMinLat(){ return minLat; }
      int getMinLon(){ return minLon; }
      int getMaxLat(){ return maxLat; }
      int getMaxLon(){ return maxLon; }
      int getDegLat(){ return degLat; }
      int getDegLon(){ return degLon; }
};
#endif
