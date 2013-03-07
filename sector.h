/* sector.h
 * Created 25SEP11
 *
 * Jonathan Cox
 * Clemson University
 */

#ifndef SECTOR_H_
#define SECTOR_H_

#include <vector>

#include "Vector.h"
#include "chip.h"
#include "geoFunct.h"

class sector{
  public:
    sector(double, double, Vector2d&);
    ~sector();

    void addChip(int);
    void removeChip(int);
    void drawSectors(QGLWidget * );
    void drawValueText(QGLWidget *);

    int findChip(Vector2d & p);

  private:
    sector(sector&);
    sector& operator=(sector&);

    std::vector<std::vector<Vector2d*> *> endPairs; // x,y one end, w,z the other end
    std::vector<bool> drawEnd;
    std::vector<int> chipValues;
    std::vector<double> radius;
    Vector2d center;
    double theta1, theta2;
  
};

#endif
