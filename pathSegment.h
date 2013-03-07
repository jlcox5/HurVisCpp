/*
 * pathSegment.h 
 *
 * Created on: Apr 5, 2011
 * Jonathan Cox
 * Clemson University
 */

#include <vector>
#include "Vector.h"

#ifndef PATHSEGMENT_H_
#define PATHSEGMENT_H_

class pathSegment{
   private:
      double bDif, sDif;
      double phB, phS;
      double pmaxB, pminB;
      double pmaxS, pminS;
      double pareaB, pareaS;
      double preW;
      int samples;
      Vector2d midPoint;
      double time;
      double phB1, phB2;
      double phS1, phS2;
      double bRange, sRange;

   public:
      pathSegment(Vector4d* , Vector4d * , double , std::vector<Vector4d*> * , std::vector<Vector4d*> *, double, double);

      std::vector<double> preSpeedF;
      std::vector<double> preSpeedPos;
      std::vector<double> preSpeedDistFunct;
      std::vector<double> preBearF;
      std::vector<double> preBearPos;
      std::vector<double> preBearDistFunct;
      
      void genPreBearDif();
      void genPreSpeedDif();
      void computePreAreaB();
      void computePreAreaS();
      double findPreKB(double x, int i, double tWB1, double tWB2);
      double findPreKS(double x, int i, double tWS1, double tWS2);
      double getPreProbBear(double p);
      double getPreProbSpeed(double p);

      double getBDif(){ return bDif; }
      double getSDif(){ return sDif; }

      double getPhB(){ return phB; }
      double getPhS(){ return phS; }

      double getPMinB(){ return pminB; }
      double getPMaxB(){ return pmaxB; }
      double getPMinS(){ return pminS; }
      double getPMaxS(){ return pmaxS; }

      void BearingToFile();
};

#endif
