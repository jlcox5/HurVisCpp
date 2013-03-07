/* Jonathan Cox
   Clemson University

   The chip class for users to assign values to sectors of probability
*/

#ifndef CHIP_H_
#define CHIP_H_

#include <QGLWidget>
#include "Matrix.h"

class chip{

   private:
      Vector2d pos;
      double rad;
      double value;
      double isMoving;
      int sector, fillPos;
      Vector2d sectorPos;
      Vector2d initPos;
      bool draw;

   public:
      chip(double v, double x, double y, double r);

      void setValue(double v){ value = v; }

      double getValue(){ return value; }
      Vector2d getPos(){ return pos; }
      void setPos(Vector2d p){ pos = p; sectorPos = p; }
      void setInitPos(){ pos = initPos; sectorPos = initPos; }
      void selectChip(Vector2d p);
      int releaseChip(Vector2d p, std::vector<Vector2d*> se, std::vector<double> tl, double r);
      void moveChip(Vector2d v);
      void printPos();

      int getChipSect(){ return sector; }
      int getSectFillPos(){ return fillPos; }
      void setSectFillPos(int f){ fillPos = f; }

      void drawChip();
      void setDrawChip(bool const);
      const bool getDrawChip() const{ return draw; }

      void setSector(int i){ sector = i; }

};

#endif
