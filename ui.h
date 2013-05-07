/*
 *  ui.cpp
 *
 *  Created on: May 7, 2013
 *  Jonathan Cox
 *  Clemson University
 *
 *  This is the container class to hold all of the ui elements.  Ideally, each element should
 *  be derived from a uiElement base class, but we'll see if I can make that work shortly.  It
 *  might be a good... programming exercise. =)
 */

#ifndef UI_H_
#define UI_H_

#include <QGLWidget>

#include "chip.h"
#include "sector.h"
#include "slider.h"

class ui{
  public:
    ui();
    ~ui();

    // Build the sectors for user interaction
    void buildTargetArea();
    void buildSectors();

    // Handling the chips
    void selectChip(Vector2d p);
    void releaseChip();
    void moveChip(Vector2d v);
    int chipsPlaced();
    void reset();

    // Slider Stuff
    void checkSliderPressed(Vector2d p);
    void moveSlider(Vector2d v);
    int getSliderCurTick(){ return slider.getCurTick(); }

    // Determines lock positions of chips once inside a sector
    void detChipLockPos();

    // Returns the value at the specified sector
    int sectorValue(int i);

    // Draw stuff
    void drawTargetArea(QGLWidget *);
    void drawChips();
    void drawSectors(QGLWidget *);
    void drawChipText(QGLWidget * g);
    void drawChipBox();
    void drawSlider();

    // Print function
    void printSectors();

  private:
    // Used for user interaction
    std::vector<chip*> chips;
    std::vector<Vector2d*> sectorEnds;
    std::vector<Vector2d*> targetArea;
    std::vector<double> thetaList;
    std::vector<double> chipThetaList;
    std::vector<double> chipDistList;
    //std::vector<std::vector<int>*> sectorFull;
    std::vector<sector*> sectorList;
    Slider slider;
    double rad;

    ui(ui & rhs);
};

#endif
