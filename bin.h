/* Jonathan Cox
   Clemson University
   Bin Structure
*/

#include <vector>
#include "gridPoint.h"
#include "Vector.h"

#ifndef BIN_H_
#define BIN_H_

class bin{
   protected:
      double wB, wS;   // Difference between max and min
      double hB, hS;
      double maxB, minB;
      double maxS, minS;
      int samples;
      int hasPrePath;
      double areaB, areaS;

      double globalChosen;

      Vector2d midPoint;

   public:
      std::vector<gridPoint*> b;
      std::vector<double> predStartTime;
      std::vector<double> predFinishTime;
      std::vector<double> bearF;
      std::vector<double> bearPos;
      std::vector<double> bearDistFunct;
      std::vector<double> speedF;
      std::vector<double> speedPos;
      std::vector<double> speedDistFunct;


      // Distance to closest point for each 3 hour segment of the predicted path
      std::vector<std::vector<double>*> preSegDist;

      bin(int minLat, int minLon){ 
         hasPrePath = 0;
         midPoint.set((float)minLon - 0.5, (float)minLat + 0.5); 
         //cout << "     midpoint set: " << midPoint << endl;
      }

      void resolve();
      double findKB(double x);
      double findKS(double x);
      void genBearDif();
      void genSpeedDif();
      void computeAreaB();
      void computeAreaS();
      double getProbBear3Hour(double p, double min, int histOrPre);
      double getProbBear(double p);
      double getProbSpeed3Hour(double p, double min, int histOrPre);
      double getProbSpeed(double p);
      void BearingToFile();
      void buildSegDist(int curAdv);
      double findOmega(double min);
      double findRo(double min);
};
#endif 
