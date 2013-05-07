/* Jonathan Cox
   Clemson University

   The slider allows users to control the points of the path that are shown.
*/

// Graphics Stuff
#ifdef __APPLE__
#  include <GLUT/glut.h>
#else
#  include <GL/glut.h>
#  include <GL/glu.h>
#endif

// Standard Libs

// Custom Libs
#include "slider.h"

Slider::Slider(): curPos(225,717),
                  barPos(225,725),
                  color(0.5, 0.5, 0.5, 0.9), 
                  sizeX(10), 
                  sizeY(20),
                  barLenX(575),
                  barLenY(3),
                  dX(0),
                  curTick(0),
                  tickSpacing(25),
                  sliderPressed(false){
}

void Slider::drawSlider(){
 
  // Draw slider bar
  glColor4f(color.x, color.y, color.z, color.w);
  glBegin(GL_POLYGON);
    glVertex3f(barPos.x, barPos.y, 0.0);
    glVertex3f(barPos.x+barLenX, barPos.y, 0.0);
    glVertex3f(barPos.x+barLenX, barPos.y+barLenY, 0.0);
    glVertex3f(barPos.x, barPos.y+barLenY, 0.0);
  glEnd();

  // Draw tick marks on bar
  glLineWidth(1.0);
  glColor4f(0.00, 0.00, 0.00, color.w);
  Vector2d linePos(barPos.x, barPos.y - 7);
  for(int i = 0; i < 24; i++){
    glBegin(GL_LINES);
      glVertex3f(linePos.x, linePos.y, 0.0);
      glVertex3f(linePos.x, linePos.y+15, 0.0);
    glEnd();
    linePos.x += tickSpacing;
  }

  // Draw slider tab 
  glColor4f(color.x, color.y, color.z, color.w);
  glBegin(GL_POLYGON);
    glVertex3f(curPos.x-(sizeX/2), curPos.y, 0.0);
    glVertex3f(curPos.x+(sizeX/2), curPos.y, 0.0);
    glVertex3f(curPos.x+(sizeX/2), curPos.y+sizeY, 0.0);
    glVertex3f(curPos.x-(sizeX/2), curPos.y+sizeY, 0.0);
  glEnd();

  glLineWidth(2.0);
  glColor4f(0, 0, 0, color.w);
  glBegin(GL_LINE_LOOP);
    glVertex3f(curPos.x-(sizeX/2), curPos.y, 0.0);
    glVertex3f(curPos.x+(sizeX/2), curPos.y, 0.0);
    glVertex3f(curPos.x+(sizeX/2), curPos.y+sizeY, 0.0);
    glVertex3f(curPos.x-(sizeX/2), curPos.y+sizeY, 0.0);
  glEnd();
}

void Slider::checkSliderPressed(Vector2d p){
  if(p.x < curPos.x + (sizeX/2) && p.x > curPos.x-(sizeX/2)){
    if(p.y < curPos.y + sizeY && p.y > curPos.y){
      cout << "Pressing the slider!" << endl;
      sliderPressed = true;
      return;
    }
  }
  sliderPressed = false;
  return;
}

void Slider::moveSlider(Vector2d p){
  if(sliderPressed){
    // Determine new position
    curTick = ((int)p.x-(int)barPos.x)/tickSpacing;
    int partialTick = ((int)p.x-(int)barPos.x)%tickSpacing;

    if(partialTick >= tickSpacing/2){
      curTick++;
    }
    curTick = max(curTick, 0);
    curTick = min(curTick, 24);
    curPos.x = min(barPos.x+curTick*tickSpacing, barPos.x+barLenX);
  }
}

void Slider::reset(){
  curPos.set(225,717);
  barPos.set(225,725);
  dX = 0;
  curTick = 0;
  sliderPressed = false;
}
