/* Jonathan Cox
   Clemson University

   The slider allows users to control the points of the path that are shown
*/


#ifndef SLIDER_H_
#define SLIDER_H_

// Standard Libs

// Custom Libs
#include "Matrix.h" 

class Slider{
  public:
    Slider();
    ~Slider(){};

   void drawSlider();
   void checkSliderPressed(Vector2d);
   void moveSlider(Vector2d);

   int getCurTick(){ return curTick; }
   void reset();

  private:
    Vector2d curPos;
    Vector2d barPos;
    Vector4d color;
    double sizeX, sizeY;
    double barLenX, barLenY;
    int dX;
    int curTick;
    int tickSpacing;
    bool sliderPressed;

    // Shouldn't copy a slider
    Slider(Slider const & s);

};

#endif
